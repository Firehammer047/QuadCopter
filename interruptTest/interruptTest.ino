#include<Servo.h>

Servo motor1,motor2,motor3,motor4;

int input[5];
int last_channel[5];
unsigned long timer[7];

int ch1 = input[0]; //yaw
int ch2 = input[1]; //throttle
int ch3 = input[2]; //pitch
int ch4 = input[3]; //roll
int ch5 = input[4]; //Aux Channel, see controller
int ch6 = input[5]; //Aux Channel, see controller

void setup() {
  Serial.begin(57600);
  enableInterrupt();

  motor1.attach(3);
  motor2.attach(4);
  motor3.attach(5);
  motor4.attach(6);
}

void loop() {
  ch1 = input[0]; //yaw
  ch2 = input[1]; //throttle
  ch3 = input[2]; //pitch
  ch4 = input[3]; //roll

  motor1.write(ch1);
  motor2.write(ch2);
  motor3.write(ch3);
  motor4.write(ch4);

  

  Serial.print(ch1);
  Serial.print("\t");
  Serial.print(ch2);
  Serial.print("\t");
  Serial.print(ch3);
  Serial.print("\t");
  Serial.print(ch4);
  Serial.println();
  
  
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



