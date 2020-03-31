/*	
	Quadcopter main FC firmware
	Developed by GB Tony Cabrera and C McCormick
	Copyright (c) 2016-2017 WVU Robotics Club
	Original code except for included libraries
*/

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#include<Servo.h>
#include<Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
// #include "MPU6050.h" 

#define USE_GYRO true

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 9
#define MAX_ANGLE 50

#define X_GYRO_OFF 220	//TODO find these values and correct
#define Y_GYRO_OFF 76
#define Z_GYRO_OFF -85    // -85
#define Z_ACC_OFF 1788

#define KPr 1
#define KIr 1
#define KDr 1

#define KPp 1
#define KIp 1
#define KDp 1

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
	mpuInterrupt = true;
}

Servo motor1, motor2, motor3, motor4;

int Motor1Port = 3;
int Motor2Port = 4;
int Motor3Port = 5;
int Motor4Port = 6;

int ESCTrigger = 20; //The value that the ESCs unlock the motors at with a 10ms pulse

//Control Authority Variables

float roll_authority = 0.25;
float pitch_authority = 0.25;
float yaw_authority = 0.25;
float throttle_authority = 0.6;

float MOTOR_1;
float MOTOR_2;
float MOTOR_3;
float MOTOR_4;

//Declaring some global variables
//http://www.brokking.net/imu.html
//That's where I got the code -Conor
int gyro_x, gyro_y, gyro_z;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
long loop_timer;
float angle_pitch_output, angle_roll_output;

float targetRoll; //Get this variable by calling getTargetRoll();
float targetPitch; //Get this variable by calling getTargetPitch();

float throttle;
float yaw;


float currentRoll;
float RollError;
float lastRollError;
float RollDerivative;
float RollIntegral;
float roll;

float currentPitch;
float PitchError;
float lastPitchError;
float PitchDerivative;
float PitchIntegral;
float pitch;

unsigned long pitchTime;
unsigned long lastPitchTime;
unsigned long rollTime;
unsigned long lastRollTime;

int input[3];
int last_channel[4];
unsigned long timer[3];

int ch1 = 0; //yaw
int ch2 = 0; //throttle
int ch3 = 0; //pitch
int ch4 = 0; //roll

float getTargetRoll() {
  //Use all the stuff from the Interrupt File in order to calculate the angles that we need

  int target_roll;

  int ch_roll = ch4 - 1500;
  float v1 = ch_roll / 500;
  float target_roll_angle = v1 * MAX_ANGLE;

  return target_roll_angle;

}

float getTargetPitch() {

  int target_pitch;

  int ch_pitch = ch3 - 1500;
  float v2 = ch_pitch / 500;
  float target_pitch_angle = v2 * MAX_ANGLE;

  return target_pitch_angle;

}

void setup() {
	pinMode(LED_PIN, OUTPUT);
	digitalWrite(LED_PIN, HIGH);	//Set LED high to indicate startup

	motor1.attach(Motor1Port);
	motor2.attach(Motor2Port);
	motor3.attach(Motor3Port);
	motor4.attach(Motor4Port);

	Wire.begin();
	Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
	Serial.begin(115200);
	// initialize device
	Serial.println(F("Initializing I2C devices..."));
	mpu.initialize();
	pinMode(INTERRUPT_PIN, INPUT);
    
	// verify connection
	Serial.println(F("Testing device connections..."));
	Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    
	// load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(X_GYRO_OFF);
    mpu.setYGyroOffset(Y_GYRO_OFF);
    mpu.setZGyroOffset(Z_GYRO_OFF);
    mpu.setZAccelOffset(Z_ACC_OFF); 

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }


/*
  for (int cal_int = 0; cal_int < 2000 ; cal_int ++) {                 //Run this code 2000 times
    read_mpu_6050_data();                                              //Read the raw acc and gyro data from the MPU-6050
    gyro_x_cal += gyro_x;                                              //Add the gyro x-axis offset to the gyro_x_cal variable
    gyro_y_cal += gyro_y;                                              //Add the gyro y-axis offset to the gyro_y_cal variable
    gyro_z_cal += gyro_z;                                              //Add the gyro z-axis offset to the gyro_z_cal variable
    delay(3);                                                          //Delay 3us to simulate the 250Hz program loop
  }
  gyro_x_cal /= 2000;                                                  //Divide the gyro_x_cal variable by 2000 to get the avarage offset
  gyro_y_cal /= 2000;                                                  //Divide the gyro_y_cal variable by 2000 to get the avarage offset
  gyro_z_cal /= 2000;                                                  //Divide the gyro_z_cal variable by 2000 to get the avarage offset

*/
  
  digitalWrite(LED_PIN, LOW);                                               //All done, turn the LED off

  loop_timer = micros();                                               //Reset the loop timer

  enableInterrupt();
  delay(3);

  startup(); //Activates Motors

}

void loop() {

  ch1 = input[0]; //yaw
  ch2 = input[1]; //throttle
  ch3 = input[2]; //pitch
  ch4 = input[3]; //roll

//  Serial.print(ch1);
//  Serial.print("\t");
//  Serial.print(ch2);
//  Serial.print("\t");
//  Serial.print(ch3);
//  Serial.print("\t");
//  Serial.print(ch4);
//  Serial.print("\t");
  //Serial.println();

  //enablePID

  if (USE_GYRO) {

	// if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        Serial.print("ypr\t");
        Serial.print(ypr[0] * 180/M_PI);
        Serial.print("\t");
        Serial.print(ypr[1] * 180/M_PI);
        Serial.print("\t");
        Serial.println(ypr[2] * 180/M_PI);
    }


    angle_pitch_output = ypr[1] * 180/M_PI;
    angle_roll_output = ypr[2] * 180/M_PI;

    //PID for Pitch

    currentPitch = angle_pitch_output;
    pitchTime = millis();

    PitchError = targetPitch - currentPitch;

    PitchDerivative = PitchError - lastPitchError;

    PitchIntegral += PitchError * (pitchTime - lastPitchTime);

    if (PitchIntegral * KIp > 1000) {
      PitchIntegral = 1000 / KIp;
    }

    pitch = PitchError * KPp + PitchDerivative * KDp + PitchIntegral * KIp;

    lastPitchError = PitchError;
    lastPitchTime = pitchTime;


    //PID for Roll

    targetRoll = getTargetRoll();
    currentRoll = angle_roll_output;
    rollTime = millis();

    RollError = targetRoll - currentRoll;

    RollDerivative = RollError - lastRollError;

    RollIntegral += RollError * (rollTime - lastRollTime);

    if (RollIntegral * KIr > 1000) {
      RollIntegral = 1000 / KIr;
    }

    roll = currentPitch * KPr + RollDerivative * KDr + RollIntegral * KIr;

    lastRollError = RollError;
    lastRollTime = rollTime;

    throttle = ch2 - 1000;
    yaw = ch1 - 1500;

    MOTOR_1 = 1000 + throttle * throttle_authority + roll + yaw + pitch;
    MOTOR_2 = 1000 + throttle * throttle_authority - roll - yaw + pitch;
    MOTOR_3 = 1000 + throttle * throttle_authority - roll + yaw - pitch;
    MOTOR_4 = 1000 + throttle * throttle_authority + roll - yaw - pitch;

    if (MOTOR_1 > 2000) {
      MOTOR_1 = 2000;
    } else if (MOTOR_1 < 1000) {
      MOTOR_1 = 1000;
    }
    if (MOTOR_2 > 2000) {
      MOTOR_2 = 2000;
    } else if (MOTOR_2 < 1000) {
      MOTOR_2 = 1000;
    }
    if (MOTOR_3 > 2000) {
      MOTOR_3 = 2000;
    } else if (MOTOR_3 < 1000) {
      MOTOR_3 = 1000;
    }
    if (MOTOR_4 > 2000) {
      MOTOR_4 = 2000;
    } else if (MOTOR_4 < 1000) {
      MOTOR_4 = 1000;
    }


//    Serial.print(MOTOR_1);
//    Serial.print("\t");
//    Serial.print(MOTOR_2);
//    Serial.print("\t");
//    Serial.print(MOTOR_3);
//    Serial.print("\t");
//    Serial.print(MOTOR_4);
//    Serial.println();

    motor1.writeMicroseconds(MOTOR_1);
    motor2.writeMicroseconds(MOTOR_2);
    motor3.writeMicroseconds(MOTOR_3);
    motor4.writeMicroseconds(MOTOR_4);

  } else {

    throttle = ch2 - 1000;
    roll = ch4 - 1500;
    pitch = ch3 - 1500;
    yaw = ch1 - 1500;

    MOTOR_1 = 1000 + throttle_authority * throttle - roll_authority * roll + yaw_authority * yaw - pitch_authority * pitch;
    MOTOR_2 = 1000 + throttle_authority * throttle + roll_authority * roll - yaw_authority * yaw - pitch_authority * pitch;
    MOTOR_3 = 1000 + throttle_authority * throttle + roll_authority * roll + yaw_authority * yaw + pitch_authority * pitch;
    MOTOR_4 = 1000 + throttle_authority * throttle - roll_authority * roll - yaw_authority * yaw + pitch_authority * pitch;

    if (MOTOR_1 > 2000) {
      MOTOR_1 = 2000;
    } else if (MOTOR_1 < 1000) {
      MOTOR_1 = 1000;
    }
    if (MOTOR_2 > 2000) {
      MOTOR_2 = 2000;
    } else if (MOTOR_2 < 1000) {
      MOTOR_2 = 1000;
    }
    if (MOTOR_3 > 2000) {
      MOTOR_3 = 2000;
    } else if (MOTOR_3 < 1000) {
      MOTOR_3 = 1000;
    }
    if (MOTOR_4 > 2000) {
      MOTOR_4 = 2000;
    } else if (MOTOR_4 < 1000) {
      MOTOR_4 = 1000;
    }

    motor1.writeMicroseconds(MOTOR_1);
    motor2.writeMicroseconds(MOTOR_2);
    motor3.writeMicroseconds(MOTOR_3);
    motor4.writeMicroseconds(MOTOR_4);

//    Serial.print(MOTOR_1);
//    Serial.print("\t");
//    Serial.print(MOTOR_2);
//    Serial.print("\t");
//    Serial.print(MOTOR_3);
//    Serial.print("\t");
//    Serial.print(MOTOR_4);
//    Serial.println();
  }

//  while (micros() - loop_timer < 4000);                                //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop
//  loop_timer = micros();                                               //Reset the loop timer
}

void startup() {
  motor1.write(ESCTrigger);
  motor2.write(ESCTrigger);
  motor3.write(ESCTrigger);
  motor4.write(ESCTrigger);
  delay(10);
  motor1.write(0);
  motor2.write(0);
  motor3.write(0);
  motor4.write(0);
}

void enableInterrupt() {
  PCICR |= (1 << PCIE0);//Engables Register 0
  PCMSK0 |= (1 << PCINT0); //Digital Port 7 on Arduino Uno
  PCMSK0 |= (1 << PCINT1); //Digital Port 8 on Arduino Uno
  PCMSK0 |= (1 << PCINT2); //Digital Port 9 on Arduino Uno
  PCMSK0 |= (1 << PCINT3); //Digital Port 10 on Arduino Uno
  PCMSK0 |= (1 << PCINT4); //Digital Port 11 on Arduino Uno
  //Tells Registers that Interrupts are enabled
}

ISR(PCINT0_vect) {
  timer[0] = micros();
  //Channel 1
  if (last_channel[0] == 0 && PINB & B00000001) {
    last_channel[0] = 1;
    timer[1] = timer[0];
  }
  else if (last_channel[0] == 1 && !(PINB & B00000001)) {
    last_channel[0] = 0;
    input[0] = timer[0] - timer[1];
  }
  //Channel 2
  if (last_channel[1] == 0 && PINB & B00000010) {
    last_channel[1] = 1;
    timer[2] = timer[0];
  }
  else if (last_channel[1] == 1 && !(PINB & B00000010)) {
    last_channel[1] = 0;
    input[1] = timer[0] - timer[2];
  }
  //Channel 3
  if (last_channel[2] == 0 && PINB & B00000100) {
    last_channel[2] = 1;
    timer[3] = timer[0];
  }
  else if (last_channel[2] == 1 && !(PINB & B00000100)) {
    last_channel[2] = 0;
    input[2] = timer[0] - timer[3];
  }
  //Channel 4
  if (last_channel[3] == 0 && PINB & B00001000) {
    last_channel[3] = 1;
    timer[4] = timer[0];
  }
  else if (last_channel[3] == 1 && !(PINB & B00001000)) {
    last_channel[3] = 0;
    input[3] = timer[0] - timer[4];
  }

}
