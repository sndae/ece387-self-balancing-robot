# Materials #
  * (1)  Arduino Uno----------------------------------------------------------$27.95 ([Amazon](http://www.amazon.com/Arduino-UNO-board-DIP-ATmega328P/dp/B006H06TVG/ref=sr_1_1?ie=UTF8&qid=1399763525&sr=8-1&keywords=uno+arduino))

  * (1)  L293d H-Bridge/motor shield-----------------------------------$16.20 ([Amazon](http://www.amazon.com/SainSmart-L293D-Shield-Arduino-Duemilanove/dp/B00813HBBO))

  * (4) brass motherboard standoffs to mount Arduino)------------$0.25 ea

  * (4) 1/4" M3 screws (to mount Arduino)-----------------------------$0.10 ea

  * (2)  1.5" diameter 12.6V DC Motors----------------------------------$10 each (eBay)

  * (1)  ADXL345 3-axis accelerometer-----------------------------------$2.50 (eBay)/$17.95 ([Sparkfun](https://www.sparkfun.com/products/9836))

  * (1)  ITG3205 3-axis gyroscope-----------------------------------------$4.98 (eBay)/$24.95 ([Sparkfun](https://www.sparkfun.com/products/11977))

  * (1)  breadboard----------------------------------------------------------------$3.50 (eBay)

  * (3)  3"x8"x0.5" wood sheets----------------------------------------------$0.33 ea (Home Depot)

  * (8)  1/4" x 6" bolts-----------------------------------------------------------$0.26 ea (Home Depot)

  * (12) 1/4" nuts-----------------------------------------------------------------$0.06 ea (Home Depot)

  * (1)  12V power supply-------------------------------------------------------$20

  * (4)  1.5" brass U-brackets + screws to mount motors--------------$0.37 ea (Home Depot)

  * (2) 2" rubber wheels (good grip)------------------------------------------$2.50 ea (Walmart)

  * a few feet of 20 AWG wire

  * some superglue

  * bundle of zip ties (to hold breadboard)---------------------------------$0.50


Total Cost: $109.31 (using eBay chips)/$142.73 (using Sparkfun chips)

# Useful tools #

  * Soldering iron

  * mini flat head screw driver

  * power drill

# Design Process and Algorithms #
### Physical Build ###
To make the 3-tier platform, four holes were drilled near the corners on each sheet and then bolted together with the 6" bolts and nuts.

![http://i.imgur.com/pRS50Fe.jpg](http://i.imgur.com/pRS50Fe.jpg)

The two 12V DC motors were mounted on the bottom of the lowest platform using the brass U-brackets and small screws.

![http://i.imgur.com/Altk1f9.jpg](http://i.imgur.com/Altk1f9.jpg)

The Arduino Uno was on top of the lowest platform in the center using the brass standoffs.  Small holes were drilled into the wood where the Arduino mounting hols are and the standoffs were glued into place.  That way the Arduino could be easily removed from the robot for whatever reason.


The breadboard was mounted next to the Arduino using zip ties.  The L293d motor shield was mounted on top of the Arduino.  Because DC motors only turn in one direction when a voltage is applied, an L293d H-bridge motor shield, which can switch a motor's polarity, was used.  The shield routed power to both motors and the Arduino.

![http://i.imgur.com/Altk1f9.jpg](http://i.imgur.com/Altk1f9.jpg)

The two leads for each motor were put into the M1 and M2 terminals on the shield.  The Vcc and GND wires from the power supply were attached the the Vcc and GND termials on the shield as well.

![http://i.imgur.com/HoxyWfN.jpg](http://i.imgur.com/HoxyWfN.jpg)

The accelerometer and gyroscope were placed on the breadboard and wired up to the 3.3V pin and ground pins on the Arduino.  The SDA pins were attached to A1 and the SCL pins were attached to A0.  Please note the direction in which the accelerometer and gyroscope axes point, as these dictate which angles are calculated in the Arduino script.  Their directions are different because their pins are located along different axes.

![http://i.imgur.com/bflQxMc.jpg](http://i.imgur.com/bflQxMc.jpg)




### Method for Balancing the Robot ###

The balancing was done by calculating the angle of tilt the robot experiences at any given time, then telling the motors to counteract that angle.

The accelerometer was used to get a rough estimate of this angle.  Because it is prone to error due to any bounces or quick movements, the ITG 3205 gyroscope was used.  This gyroscope, which measures change in degrees per second, was used to accurately determine how far the robot tilted.  This combination allowed us to find the robot's true current angle.

From this angle, the motors were assigned speed values in order to bring the robot's current angle back to zero (vertical).

# Arduino Script #

### Variables and Initialization ###
This sections defines all the variables, sensor addresses, constants and libraries used in the program

```
//------------------------------------------------------------------------
// Arduino pins for the shift register
#define MOTORLATCH  12 // changed from 12
#define MOTORCLK    4
#define MOTORENABLE 7
#define MOTORDATA   8
// 8-bit bus after the 74HC595 shift register
// (not Arduino pins)
// These are used to set the direction of the bridge driver.
#define MOTOR1_A 2
#define MOTOR1_B 3
#define MOTOR2_A 1
#define MOTOR2_B 4
// Arduino pins for the PWM signals.
#define MOTOR1_PWM 11 // changed from 11
#define MOTOR2_PWM 3
// Codes for the motor function.
#define FORWARD  1
#define BACKWARD 2
#define BRAKE    3
#define RELEASE  4

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

int toggleDirection = 1; // inidicates motor is going forward
int motor1offset = 0;
int motor2offset = 5;
int speed1 = 0;
int speed2 = 0;
int motorMultiplier = 50; // larger = less dead area but quicker speed cap
int deadZoneRemover = 80; // removes dead zone when vertical
double centerZone = 0.2; // +/- q is the zone in the center where motors will be stopped
// speed thresholds and speed multiples
double speedThresh[] = {2, 6, 8, 12};
double x[] = {0.13, 0.16, 0.21, 0.3, 5};

double unsignedEvent, signedEvent;
// give the accelerometer thing a name (accel)
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

//-------------------------------------------------------------------
//-------------------------------------------------------------------
//This is a list of registers in the ITG-3205. Registers are parameters that determine how the sensor will behave, or they can hold data that represent the
//sensors current status.
//To learn more about the registers on the ITG-3205, download and read the datasheet.
char WHO_AM_I = 0x00;
char SMPLRT_DIV= 0x15;
char DLPF_FS = 0x16;
char GYRO_XOUT_H = 0x1D;
char GYRO_XOUT_L = 0x1E;
char GYRO_YOUT_H = 0x1F;
char GYRO_YOUT_L = 0x20;
char GYRO_ZOUT_H = 0x21;
char GYRO_ZOUT_L = 0x22;

//This is a list of settings that can be loaded into the registers.
//DLPF, Full Scale Register Bits
//FS_SEL must be set to 3 for proper operation
//Set DLPF_CFG to 3 for 1kHz Fint and 42 Hz Low Pass Filter
char DLPF_CFG_0 = 1<<0;
char DLPF_CFG_1 = 1<<1;
char DLPF_CFG_2 = 1<<2;
char DLPF_FS_SEL_0 = 1<<3;
char DLPF_FS_SEL_1 = 1<<4;

int   STD_LOOP_TIME  = 9;
int lastLoopTime = STD_LOOP_TIME;
int lastLoopUsefulTime = STD_LOOP_TIME;
unsigned long loopStartTime = 0;
double actAngle = 0;
double dt = 0;
double x_angle = 0;
double xGyroBias = 6.35;
double yGyroBias = -1.88;
double zGyroBias = -2.7;
//Create variables to hold the gyro output rates.
double xGyroRate, yGyroRate, zGyroRate;

//Kalman
double P00=0;
double P01=0;
double P10=0;
double P11=0;
double K0=0;
double K1=1;
double bias=0;

//I2C devices each have an address. The address is defined in the datasheet for the device. The ITG-3200 breakout board can have different address depending on how
//the jumper on top of the board is configured. By default, the jumper is connected to the VDD pin. When the jumper is connected to the VDD pin the I2C address
//is 0x69.

char itgAddress = 0x69;
```

### Setup ###
```
---------------------------------------------------------------------------
void setup()
{
Serial.begin(115200);
Serial.println("Accelerometer Test"); Serial.println("");
/* Initialise the sensor */
if(!accel.begin())
{
Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
while(1);
}
accel.setRange(ADXL345_RANGE_16_G);  /* Set the range to whatever is appropriate for your project */
displaySensorDetails();   /* Display some basic information on this sensor */
displayRange();  /* Display additional settings (outside the scope of sensor_t) */
Serial.println("");
int toggleDirection = 1; // 1: motors going forward, 0: motors going backward

//-------------------------------------------------------------------
//-------------------------------------------------------------------
//Initialize the I2C communication. This will set the Arduino up as the 'Master' device.
Wire.begin();

//Read the WHO_AM_I register and print the result
char id=0;
id = itgRead(itgAddress, 0x00);
Serial.print("ID: ");
Serial.println(id, HEX);

//Configure the gyroscope
//Set the gyroscope scale for the outputs to +/-2000 degrees per second
itgWrite(itgAddress, DLPF_FS, (DLPF_FS_SEL_0|DLPF_FS_SEL_1|DLPF_CFG_0));
//Set the sample rate to 100 hz
itgWrite(itgAddress, SMPLRT_DIV, 9);
actAngle = 0;
//-------------------------------------------------------------------
//-------------------------------------------------------------------

}
```

### Loop ###
Functionality is provided to the program by the data extracted and used in the methods of this section
```

void loop()
{

//-------------------------------------------------------------------
//-------------------------------------------------------------------
// ADXL345 readings
sensors_event_t ADXLevent1;
sensors_event_t ADXLevent2;
sensors_event_t ADXLevent3;
// get the 3 events
accel.getEvent(&ADXLevent1);
//delay(1);
accel.getEvent(&ADXLevent2);
//delay(1);
accel.getEvent(&ADXLevent3);


//Read the x,y and z output rates from the gyroscope.
xGyroRate = itgReadX() / 14.375 - xGyroBias;
yGyroRate = itgReadY() / 14.375 - yGyroBias;
zGyroRate = itgReadZ() / 14.375 - zGyroBias;

actAngle = getDegree(xGyroRate, lastLoopTime);

//Print the output rates to the terminal, seperated by a TAB character.
Serial.print("gyro (x,y,z): ");
Serial.print(xGyroRate);
Serial.print('\t');
Serial.print(yGyroRate);
Serial.print('\t');
Serial.print(zGyroRate);

Serial.print('\t');
Serial.print("angle: ");
Serial.print(actAngle);
Serial.print('\t');

Serial.print("accel: ");
Serial.print(ADXLevent1.acceleration.x);
Serial.print('\t');
Serial.print("motors: ");
//-------------------------------------------------------------------
//-------------------------------------------------------------------



// calculate speeds based on the events
unsignedEvent = abs(actAngle);
signedEvent = actAngle;
speed1 = motorMultiplier*unsignedEvent+motor1offset;
speed2 = motorMultiplier*unsignedEvent+motor2offset;

// robot in upright position:  shut motors off
if(unsignedEvent < centerZone) {
motor(1, RELEASE, 0);
motor(2, RELEASE, 0);
Serial.print('\n');
}
// robot tilting forward and moving forward
else if(signedEvent <= -centerZone && toggleDirection == 1)  {
if (unsignedEvent < speedThresh[0]) {
forwardSpeed1();
}
else if (unsignedEvent >= speedThresh[0] && unsignedEvent < speedThresh[1]) {
forwardSpeed2();
}
else if (unsignedEvent >= speedThresh[1] && unsignedEvent < speedThresh[2]) {
forwardSpeed3();
}
else if (unsignedEvent >= speedThresh[2] && unsignedEvent < speedThresh[3]) {
forwardSpeed4();
}
else {
forwardSpeed5();
}
}
// robot tilting forward and moving backward
else if(signedEvent <= -centerZone && toggleDirection == 0)  {
motor(1, BRAKE, 0);
motor(2, BRAKE, 0);
delay(10);
if (unsignedEvent < speedThresh[0]) {
forwardSpeed1();
}
else if (unsignedEvent >= speedThresh[0] && unsignedEvent < speedThresh[1]) {
forwardSpeed2();
}
else if (unsignedEvent >= speedThresh[1] && unsignedEvent < speedThresh[2]) {
forwardSpeed3();
}
else if (unsignedEvent >= speedThresh[2] && unsignedEvent < speedThresh[3]) {
forwardSpeed4();
}
else {
forwardSpeed5();
}
toggleDirection = 1;
}
// robot tilting backward and moving forward
else if(signedEvent >= centerZone && toggleDirection == 1)  {
motor(1, BRAKE, 0);
motor(2, BRAKE, 0);
delay(10);
if (unsignedEvent < speedThresh[0]) {
backwardSpeed1();
}
else if (unsignedEvent >= speedThresh[0] && unsignedEvent < speedThresh[1]) {
backwardSpeed2();
}
else if (unsignedEvent >= speedThresh[1] && unsignedEvent < speedThresh[2]) {
backwardSpeed3();
}
else if (unsignedEvent >= speedThresh[2] && unsignedEvent < speedThresh[3]) {
backwardSpeed4();
}
else {
backwardSpeed5();

}
toggleDirection = 0;
}
// robot tilting backward and going backward
else if(signedEvent >= centerZone && toggleDirection == 0) {
if (unsignedEvent < speedThresh[0]) {
backwardSpeed1();
}
else if (unsignedEvent >= speedThresh[0] && unsignedEvent < speedThresh[1]) {
backwardSpeed2();
}
else if (unsignedEvent >= speedThresh[1] && unsignedEvent < speedThresh[2]) {
backwardSpeed3();
}
else if (unsignedEvent >= speedThresh[2] && unsignedEvent < speedThresh[3]) {
backwardSpeed4();
}
else {
backwardSpeed5();
}
}
// default case: release the motors
else
{
motor(1, RELEASE, 0);
motor(2, RELEASE, 0);
}



// *********************** loop timing control **************************
lastLoopUsefulTime = millis()-loopStartTime;
if(lastLoopUsefulTime<STD_LOOP_TIME)         delay(STD_LOOP_TIME-lastLoopUsefulTime);
lastLoopTime = millis() - loopStartTime;
loopStartTime = millis();
}
```

### ITG Functions ###
```

void itgInitialize(){
//This is a list of registers in the ITG-3200. Registers are parameters that determine how the sensor will behave, or they can hold data that represent the
//sensors current status.
//To learn more about the registers on the ITG-3200, download and read the datasheet.
char WHO_AM_I = 0x00;
char SMPLRT_DIV= 0x15;
char DLPF_FS = 0x16;
char GYRO_XOUT_H = 0x1D;
char GYRO_XOUT_L = 0x1E;
char GYRO_YOUT_H = 0x1F;
char GYRO_YOUT_L = 0x20;
char GYRO_ZOUT_H = 0x21;
char GYRO_ZOUT_L = 0x22;

//This is a list of settings that can be loaded into the registers.
//DLPF, Full Scale Register Bits
//FS_SEL must be set to 3 for proper operation
//Set DLPF_CFG to 3 for 1kHz Fint and 42 Hz Low Pass Filter
char DLPF_CFG_0 = 1<<0;
char DLPF_CFG_1 = 1<<1;
char DLPF_CFG_2 = 1<<2;
char DLPF_FS_SEL_0 = 1<<3;
char DLPF_FS_SEL_1 = 1<<4;

int   STD_LOOP_TIME  =          9;
int lastLoopTime = STD_LOOP_TIME;
int lastLoopUsefulTime = STD_LOOP_TIME;
unsigned long loopStartTime = 0;
double actAngle = 0;
double dt = 0;
double x_angle;
double x_bias;

//I2C devices each have an address. The address is defined in the datasheet for the device. The ITG-3200 breakout board can have different address depending on how
//the jumper on top of the board is configured. By default, the jumper is connected to the VDD pin. When the jumper is connected to the VDD pin the I2C address
//is 0x69.
char itgAddress = 0x69;
}

//------------------------------------------------------------------------------
//This function will write a value to a register on the itg-3200.
//Parameters:
//  char address: The I2C address of the sensor. For the ITG-3200 breakout the address is 0x69.
//  char registerAddress: The address of the register on the sensor that should be written to.
//  char data: The value to be written to the specified register.
void itgWrite(char address, char registerAddress, char data)
{
//Initiate a communication sequence with the desired i2c device
Wire.beginTransmission(address);
//Tell the I2C address which register we are writing to
Wire.write(registerAddress);
//Send the value to write to the specified register
Wire.write(data);
//End the communication sequence
Wire.endTransmission();
}
//------------------------------------------------------------------------------
//This function will read the data from a specified register on the ITG-3200 and return the value.
//Parameters:
//  char address: The I2C address of the sensor. For the ITG-3200 breakout the address is 0x69.
//  char registerAddress: The address of the register on the sensor that should be read
//Return:
//  unsigned char: The value currently residing in the specified register
unsigned char itgRead(char address, char registerAddress)
{
//This variable will hold the contents read from the i2c device.
unsigned char data=0;

//Send the register address to be read.
Wire.beginTransmission(address);
//Send the Register Address
Wire.write(registerAddress);
//End the communication sequence.
Wire.endTransmission();

//Ask the I2C device for data
Wire.beginTransmission(address);
Wire.requestFrom(address, 1);

//Wait for a response from the I2C device
if(Wire.available()){
//Save the data sent from the I2C device
data = Wire.read();
}

//End the communication sequence.
Wire.endTransmission();

//Return the data read during the operation
return data;
}
//------------------------------------------------------------------------------
//This function is used to read the X-Axis rate of the gyroscope. The function returns the ADC value from the Gyroscope
//NOTE: This value is NOT in degrees per second.
//Usage: int xRate = readX();
int itgReadX(void)
{
int data=0;
data = itgRead(itgAddress, GYRO_XOUT_H)<<8;
data |= itgRead(itgAddress, GYRO_XOUT_L);

return data;
}
//This function is used to read the Y-Axis rate of the gyroscope. The function returns the ADC value from the Gyroscope
//NOTE: This value is NOT in degrees per second.
//Usage: int yRate = readY();
int itgReadY(void)
{
int data=0;
data = itgRead(itgAddress, GYRO_YOUT_H)<<8;
data |= itgRead(itgAddress, GYRO_YOUT_L);

return data;
}
//This function is used to read the Z-Axis rate of the gyroscope. The function returns the ADC value from the Gyroscope
//NOTE: This value is NOT in degrees per second.
//Usage: int zRate = readZ();
int itgReadZ(void)
{
int data=0;
data = itgRead(itgAddress, GYRO_ZOUT_H)<<8;
data |= itgRead(itgAddress, GYRO_ZOUT_L);

return data;
}
```

### Motor Functions ###
```

//------------------------------------------------------------------------------
// motor
//
// Select the motor (1-2), the command,
// and the speed (0-255).
// The commands are: FORWARD, BACKWARD, BRAKE, RELEASE.
void motor(int nMotor, int command, int speed)
{
int motorA, motorB;

if (nMotor >= 1 && nMotor <= 2)
{
switch (nMotor)
{
case 1:
motorA   = MOTOR1_A;
motorB   = MOTOR1_B;
//Serial.println("motor 1");
break;
case 2:
motorA   = MOTOR2_A;
motorB   = MOTOR2_B;
//Serial.println("motor 2");
break;
default:
break;
}

switch (command)
{
case FORWARD:
motor_output (motorA, HIGH, speed);
motor_output (motorB, LOW, -1);     // -1: no PWM set
break;
case BACKWARD:
motor_output (motorA, LOW, speed);
motor_output (motorB, HIGH, -1);    // -1: no PWM set
break;
case BRAKE:
motor_output (motorA, LOW, 255); // 255: fully on.
motor_output (motorB, LOW, -1);  // -1: no PWM set
break;
case RELEASE:
motor_output (motorA, LOW, 0);  // 0: output floating.
motor_output (motorB, LOW, -1); // -1: no PWM set
break;
default:
break;
}
}
}
//------------------------------------------------------------------------------
// motor_output
//
// The function motor_ouput uses the motor driver to
// drive normal outputs like lights, relays, solenoids,
// DC motors (but not in reverse).
//
// It is also used as an internal helper function
// for the motor() function.
//
// The high_low variable should be set 'HIGH'
// to drive lights, etc.
// It can be set 'LOW', to switch it off,
// but also a 'speed' of 0 will switch it off.
//
// The 'speed' sets the PWM for 0...255, and is for
// both pins of the motor output.
//   For example, if motor 3 side 'A' is used to for a
//   dimmed light at 50% (speed is 128), also the
//   motor 3 side 'B' output will be dimmed for 50%.
// Set to 0 for completelty off (high impedance).
// Set to 255 for fully on.
// Special settings for the PWM speed:
//    Set to -1 for not setting the PWM at all.
//
void motor_output (int output, int high_low, int speed)
{
int motorPWM;

switch (output)
{
case MOTOR1_A:
case MOTOR1_B:
motorPWM = MOTOR1_PWM;
break;
case MOTOR2_A:
case MOTOR2_B:
motorPWM = MOTOR2_PWM;
break;
default:
// Use speed as error flag, -3333 = invalid output.
speed = -3333;
break;
}

if (speed != -3333)
{
// Set the direction with the shift register
// on the MotorShield, even if the speed = -1.
// In that case the direction will be set, but
// not the PWM.
shiftWrite(output, high_low);

// set PWM only if it is valid
if (speed >= 0 && speed <= 255)
{
analogWrite(motorPWM, speed);
}
}
}

//------------------------------------------------------------------------------
// shiftWrite
//
// The parameters are just like digitalWrite().
//
// The output is the pin 0...7 (the pin behind
// the shift register).
// The second parameter is HIGH or LOW.
//
// There is no initialization function.
// Initialization is automatically done at the first
// time it is used.
//
void shiftWrite(int output, int high_low)
{
static int latch_copy;
static int shift_register_initialized = false;

// Do the initialization on the fly,
// at the first time it is used.
if (!shift_register_initialized)
{
// Set pins for shift register to output
pinMode(MOTORLATCH, OUTPUT);
pinMode(MOTORENABLE, OUTPUT);
pinMode(MOTORDATA, OUTPUT);
pinMode(MOTORCLK, OUTPUT);

// Set pins for shift register to default value (low);
digitalWrite(MOTORDATA, LOW);
digitalWrite(MOTORLATCH, LOW);
digitalWrite(MOTORCLK, LOW);
// Enable the shift register, set Enable pin Low.
digitalWrite(MOTORENABLE, LOW);

// start with all outputs (of the shift register) low
latch_copy = 0;

shift_register_initialized = true;
}

// The defines HIGH and LOW are 1 and 0.
// So this is valid.
bitWrite(latch_copy, output, high_low);

// Use the default Arduino 'shiftOut()' function to
// shift the bits with the MOTORCLK as clock pulse.
// The 74HC595 shiftregister wants the MSB first.
// After that, generate a latch pulse with MOTORLATCH.
shiftOut(MOTORDATA, MOTORCLK, MSBFIRST, latch_copy);
delayMicroseconds(5);    // For safety, not really needed.
digitalWrite(MOTORLATCH, HIGH);
delayMicroseconds(5);    // For safety, not really needed.
digitalWrite(MOTORLATCH, LOW);
}
```

### ADXL Functions ###
```


void displaySensorDetails(void)
{
sensor_t sensor;
accel.getSensor(&sensor);
Serial.println("------------------------------------");
Serial.print  ("Sensor:       "); Serial.println(sensor.name);
Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" m/s^2");
Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" m/s^2");
Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" m/s^2");
Serial.println("------------------------------------");
Serial.println("");
delay(500);
}
//------------------------------------------------------------------------------
void displayRange(void)
{
Serial.print  ("Range:         +/- ");

switch(accel.getRange())
{
case ADXL345_RANGE_16_G:
Serial.print  ("16 ");
break;
case ADXL345_RANGE_8_G:
Serial.print  ("8 ");
break;
case ADXL345_RANGE_4_G:
Serial.print  ("4 ");
break;
case ADXL345_RANGE_2_G:
Serial.print  ("2 ");
break;
default:
Serial.print  ("?? ");
break;
}
Serial.println(" g");
}
```


### Motor Control ###
Because DC motors only turn in one direction when a voltage is applied, an L293d H-bridge motor shield, which can switch a motor's polarity, was used.  The shield routed power to both motors and the Arduino.

```
//------------------------------------------------------------------------------
// motor speed assignment functions
void forwardSpeed1() {
motor(1, FORWARD, x[0]*speed1+deadZoneRemover);
motor(2, FORWARD, x[0]*speed2+deadZoneRemover);
Serial.print("F1 ");
Serial.println(x[0]*speed1+deadZoneRemover);
}}//------------------------------------------------------------------------------
// motor speed assignment functions
void forwardSpeed1() {
motor(1, FORWARD, x[0]*speed1+deadZoneRemover);
motor(2, FORWARD, x[0]*speed2+deadZoneRemover);
Serial.print("F1 ");
Serial.println(x[0]*speed1+deadZoneRemover);
}
void forwardSpeed2() {
motor(1, FORWARD, x[1]*speed1+deadZoneRemover);
motor(2, FORWARD, x[1]*speed2+deadZoneRemover);
Serial.print("F2 ");
Serial.println(x[1]*speed1+deadZoneRemover);
}
void forwardSpeed3() {
motor(1, FORWARD, x[2]*speed1+deadZoneRemover);
motor(2, FORWARD, x[2]*speed2+deadZoneRemover);
Serial.print("F3 ");
Serial.println(x[2]*speed1+deadZoneRemover);
}
void forwardSpeed4() {
motor(1, FORWARD, x[3]*speed1+deadZoneRemover);
motor(2, FORWARD, x[3]*speed2+deadZoneRemover);
Serial.print("F4 ");
Serial.println(x[3]*speed1+deadZoneRemover);

}
void forwardSpeed5() {
motor(1, FORWARD, x[4]*speed1+deadZoneRemover);
motor(2, FORWARD, x[4]*speed2+deadZoneRemover);
Serial.print("F5 ");
Serial.println(x[4]*speed1+deadZoneRemover);

}
void backwardSpeed1() {
motor(1, BACKWARD, x[0]*speed1+deadZoneRemover);
motor(2, BACKWARD, x[0]*speed2+deadZoneRemover);
Serial.print("B1 ");
Serial.println(x[0]*speed1+deadZoneRemover);

}
void backwardSpeed2() {
motor(1, BACKWARD, x[1]*speed1+deadZoneRemover);
motor(2, BACKWARD, x[1]*speed2+deadZoneRemover);
Serial.print("B2 ");
Serial.println(x[1]*speed1+deadZoneRemover);

}
void backwardSpeed3() {
motor(1, BACKWARD, x[2]*speed1+deadZoneRemover);
motor(2, BACKWARD, x[2]*speed2+deadZoneRemover);
Serial.print("B3 ");
Serial.println(x[2]*speed1+deadZoneRemover);
}
void backwardSpeed4() {
motor(1, BACKWARD, x[3]*speed1+deadZoneRemover);
motor(2, BACKWARD, x[3]*speed2+deadZoneRemover);
Serial.print("B4 ");
Serial.println(x[3]*speed1+deadZoneRemover);
}
void backwardSpeed5() {
motor(1, BACKWARD, x[4]*speed1+deadZoneRemover);
motor(2, BACKWARD, x[4]*speed2+deadZoneRemover);
Serial.print("B5 ");
Serial.println(x[4]*speed1+deadZoneRemover);

}
```

### Angle based on Acceleration ###
To get the robot's current angle from vertical, trigonometry was used.

angle = arctan(x-acceleration reading/z-acceleration reading), where the x-axis is the line in which the robot tilts and the z-axis is the vertical axis.

<< maybe put a picture of the robot with the axis labels here (with gyroscope axis, too>>
### Angle Correction using Gyroscope ###
This rough angle from acceleration was corrected using the change in degrees per second found from the gyroscope.  This was done with the getDegree function which uses the gyroscope's rate of change in the x-direction, the elapsed time (in milliseconds) between each loop cycle, and the accelerations in the X and Z directions.

```

//------------------------------------------------------------------------------
double getDegree(float newGyroRate, int looptime, float accelerationX, float accelerationZ){
dt = float(looptime)/1000;                                    // XXXXXXX arevoir
x_angle += dt * (newGyroRate);
// fix drift based on accelerometer reading
//   if(abs(accelerationX) <=0.7 && abs(accelerationZ) >= 9.2 && abs(x_angle) >= 1.5) {
//     if(abs(accelerationX) <= .8 && abs(newGyroRate) <= 8) {
//          actAngle = x_angle = 0;
//     }
if(abs(accelerationX) <=0.7 && abs(accelerationZ) >= 9.3 && abs(newGyroRate) <= 12)
{
Serial.print("afkjadlfsdlfkjdslkfjsldkfjlkfjlksjf");
actAngle = x_angle = 0;
}
//     if(acceleration <= 8 && acceleration >= 0 && x_angle <= 10) {
//          actAngle = x_angle = 5;
//     }
//     if(acceleration >= -8 && acceleration > abs(x_angle) >= 10) {
//          actAngle = x_angle = 0;
//     }
return x_angle;
}
```

# Future Improvements #
  * More powerful motors should be used o make the robot more stable.  The motors used were not powerful enough to lift the robot up once it tilted farther than 25 degrees either way.  This would allow for us to make the robot taller.  With a taller robot, we could better use the angle made from the Kalman filter.  The reason this filtered angle was unreliable was because the Arduino could not calculate this angle fast enough for our use.  A taller robot would have a higher moment of inertia which means that the robot would take longer to tip from one side to another.  Using this angle would mean motor speed values would be more accurate and thus make the robot less jittery.

  * Using an inherently balanced design would allow the robot to balance better on its own, regardless of motor speeds.  By adding more weight down at the wheels, the motors would not have to work as hard to keep the robot balanced.

  * Use a more powerful microcontroller so that filtered angles can be computer more rapidly.  this would eliminate the ~0.25 sec lag we were experiencing, which made the filtered angle useless for our real-time application.

  * Create an LED light display that shows current acceleration of the balancing robot.  This could be built by combining 5 red LEDs on each side of a green LED to represent positive, negative, and zero valued acceleration.  A visual display of the current balancing point may help the attractiveness of the design and create a visual representation of the many calculations and adjustments made by the design.

  * After the robot is balanced there must be some way to control the robot from afar for it to be a useful design and application.  By including a way to drive the motors separately from the balancing system, more applications for the design present themselves.  An example of a control system could be radio-control like that used in an RC car.