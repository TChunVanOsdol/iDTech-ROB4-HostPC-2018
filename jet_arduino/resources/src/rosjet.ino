#include "DualVNH5019MotorShield.h"
#include "NewPing.h"
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt64.h>
#include <geometry_msgs/Twist.h>
#include "Wire.h"

/*
 * CONSTANTS
 */
//Motor Constants
#define MAX_SPEED 400
#define BRAKE_POWER 400
#define MOTOR_TIMEOUT_MS 1000

//Sonar Constants
#define SONAR_NUM 3
#define MAX_DISTANCE 200
#define PING_INTERVAL 30

//Motor Current Constants
#define CURRENT_INTERVAL 30

//Encoder Constants
#define ENCODER_INTERVAL 20

//Inertial Measurement Unit Constants
#define IMU_ADDR 0x68
#define IMU_INTERVAL 50

/*
 * GLOBAL VARIABLES
 */

//Sonar
unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
unsigned int cm[SONAR_NUM];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.

NewPing sonar[SONAR_NUM] = {     // Sensor object array.
  NewPing(51, 53, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(47, 49, MAX_DISTANCE),
  NewPing(43, 45, MAX_DISTANCE)
};
std_msgs::Int16 sonar_distance[SONAR_NUM];

ros::Publisher sonar_pub[SONAR_NUM] = {
  ros::Publisher("arduino/sonar_1", &sonar_distance[0]),
  ros::Publisher("arduino/sonar_2", &sonar_distance[1]),
  ros::Publisher("arduino/sonar_3", &sonar_distance[2])
};


//Motor Current
unsigned long currentTimer;
std_msgs::Int16 motor_right_current, motor_left_current;
ros::Publisher motor_right_current_pub("arduino/motor_right_current", &motor_right_current);
ros::Publisher motor_left_current_pub("arduino/motor_left_current", &motor_left_current);

//Motors
DualVNH5019MotorShield md;
void motor_right_speed_cb(const std_msgs::Int16 &cmd_msg);
void motor_left_speed_cb(const std_msgs::Int16 &cmd_msg);
ros::Subscriber<std_msgs::Int16> motor_right_speed_sub("arduino/motor_right_speed", motor_right_speed_cb);
ros::Subscriber<std_msgs::Int16> motor_left_speed_sub("arduino/motor_left_speed", motor_left_speed_cb);
unsigned long motorTimer;

//Ros NodeHandler
ros::NodeHandle  nh;

//Encoders
unsigned long encoderTimer;
std_msgs::UInt64 encoder_left_value, encoder_right_value;
ros::Publisher encoder_left_pub("arduino/encoder_left_value", &encoder_left_value);
ros::Publisher encoder_right_pub("arduino/encoder_right_value", &encoder_right_value);

//Inertial Measurement Unit Values
unsigned long imuTimer;
int16_t accX, accY, accZ, Tmp, gyroX, gyroY, gyroZ;
double gx, gy, gz;
geometry_msgs::Twist accel;	//Consider combining using the Accel type over Twist
geometry_msgs::Twist gyro;	//Twist holds two Vec3 messages
ros::Publisher accel_pub("arduino/accel", &accel);
ros::Publisher gyro_pub("arduino/gyro", &gyro);

/*
 * CALLBACKS
 */
void motor_right_speed_cb(const std_msgs::Int16 &cmd_msg) {
    motorTimer = millis();
    md.setM2Speed(cmd_msg.data);
    if (cmd_msg.data == 0)
      md.setM2Brake(BRAKE_POWER);
}

void motor_left_speed_cb(const std_msgs::Int16 &cmd_msg) {
    motorTimer = millis();
    md.setM1Speed(cmd_msg.data);
    if (cmd_msg.data == 0)
      md.setM1Brake(BRAKE_POWER);
}

void echoCheck() { // If ping received, set the sensor distance to array.
  if (sonar[currentSensor].check_timer())
    sonar_distance[currentSensor].data = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
}

void encoder_left_cb() {		//There are more approaches than just incrementing
  encoder_left_value.data++;	//At minimum, increments can be useful in short sample periods
}

void encoder_right_cb() {
  encoder_right_value.data++;
}

/*
 * Setup
 */

void setup() {
  //IMU startup
  Wire.begin();
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  
  //ROS startup
  md.init();
  nh.getHardware()->setBaud(115200);

  nh.initNode();

  nh.subscribe(motor_right_speed_sub);
  nh.subscribe(motor_left_speed_sub);

  nh.advertise(motor_right_current_pub);
  nh.advertise(motor_left_current_pub);

  nh.advertise(encoder_left_pub);
  nh.advertise(encoder_right_pub);

  for(uint8_t i = 0; i < SONAR_NUM; i++) {
    nh.advertise(sonar_pub[i]);
  }
  nh.advertise(accel_pub);
  nh.advertise(gyro_pub);
  
  //Initialize wait periods, avoids erratic startup values
  motorTimer = millis();

  currentTimer = millis() + 500;

  encoderTimer = millis() + 600;

  pingTimer[0] = millis() + 700;
  for (uint8_t i = 1; i < SONAR_NUM; i++) // Set the starting time for each sensor.
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;

  imuTimer = millis() + 800;

  attachInterrupt(5, encoder_left_cb, RISING); //digital pin 18
  attachInterrupt(4, encoder_right_cb, RISING); //digital pin 19
  
  //Initialize values
  encoder_left_value.data = 0;
  encoder_right_value.data = 0;

}

/*
 * Loop
 */

void loop() {

	//Sonar sensors
  for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through all the sonar sensors
    if (millis() >= pingTimer[i]) {         // Is it this sensor's time to ping?
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;  // Set next time this sensor will be pinged.
      sonar_pub[i].publish(&sonar_distance[i]);
      sonar[currentSensor].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
      currentSensor = i;                          // Sensor being accessed.
      sonar_distance[currentSensor].data = 0;                      // Make distance zero in case there's no ping echo for this sensor.
      sonar[currentSensor].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
    }
  }
	//Motor currents
  if (millis() >= currentTimer) {	//Read motor current in mA
    motor_right_current.data = md.getM2CurrentMilliamps();	//Directly set message to motor driver value
    motor_left_current.data = md.getM1CurrentMilliamps();

    motor_right_current_pub.publish(&motor_right_current);
    motor_left_current_pub.publish(&motor_left_current);
    currentTimer += CURRENT_INTERVAL;
  }
	//Motor encoders
  if (millis() >= encoderTimer) {
    encoder_left_pub.publish(&encoder_left_value); //Encoders update from interrupts
    encoder_right_pub.publish(&encoder_right_value);
    encoderTimer += ENCODER_INTERVAL;
  }
	//Motor timeouts
  if (millis() > motorTimer + MOTOR_TIMEOUT_MS) {	//Condition is met if last motor command was received
    md.setM1Brake(BRAKE_POWER);						//longer than the timeout (1 sec) period ago
    md.setM2Brake(BRAKE_POWER);
  }
    //Inertial measurement unit
  if (millis() > imuTimer) {
	Wire.beginTransmission(IMU_ADDR);	//Tell the IMU which address to start from
	Wire.write(0x3B);
	Wire.endTransmission(false);		//Use false to keep I2C bus for Arduino
	Wire.requestFrom(IMU_ADDR,14,true); //Get 14 values from IMU (upper and lower bytes for each of 7 datasets)
	//Get accelerometer data
	accX = Wire.read() << 8 | Wire.read();	
	accY = Wire.read() << 8 | Wire.read();	//Upper byte moved left, lower byte added in with OR
	accZ = Wire.read() << 8 | Wire.read();
	accel.linear.x = accX; //Needs a conversion
	//Get temperature
	Tmp = Wire.read() << 8 | Wire.read();	//Temperature is sent between accel and gyro data
	Tmp = Tmp / 340 + 36.53;				//Conversion from datasheet to Celsius
	//Get gyroscope data
	gyroX = Wire.read() << 8 | Wire.read();
	gyroY = Wire.read() << 8 | Wire.read();
	gyroZ = Wire.read() << 8 | Wire.read();
	gx = gyroX / 131.0;
	gy = gyroY / 131.0;
	gz = gyroZ / 131.0 - 1.40;
	gyro.angular.x = gx;
	gyro.angular.y = gy;
	gyro.angular.z = gz;
	//Publish
	accel_pub.publish(&accel);
	gyro_pub.publish(&gyro);
	//Increment timer
	imuTimer += IMU_INTERVAL;
  }
  
  delay(10);
  nh.spinOnce();
}
