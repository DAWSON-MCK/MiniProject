/*
 * Group 10
 * MiniProject Final Code 
 */
#include "Encoder.h"
#include <Wire.h>
#include "DualMC33926MotorShield.h"
// motor object 
DualMC33926MotorShield md;

// pin definitions for the encoder and the LCD
#define pwmMod        9 // used to modify the PWM
#define ENC1          2
#define ENC2          3
#define SLAVE_ADDRESS 0x04// dawson info  
#define TOLERANCE     20 // give some tolerance to the rotational value 
#define CPR           3200 // counts per revolution 
#define PERIOD        10

// variables used for the controller
double timeChange; // used to keep track of the time elapsed 
double error = 0; // used to calculate difference in target and current angular position
double out = 0;
double voltage = 0;
double pwmOut = 0;
float integrator = 0; // used to keep track of the accumulating error 

Encoder encoderValues(ENC1, ENC2); // used to keep track of the position of the encoder 

float oldPos; // used for delta U?
float newPos = 0;

float  currentPosition = 0; // keeps track of our encoder values 

int loopCounts; 
int desiredPos = 3; // keeps track of our quadrants 
double rad = 0; // calculates the position on the quadrant 


// here are the controller variables 
double Kp = 0.62655794027008;
double Ki = 0.139513658834803;

// from the motor shield example code
// reads from the pin to see if there is an error 

void stopIfFault()
{
  if (md.getFault())
  {
    Serial.println("fault");
    while(1);
  }
}

void setup() {
  pinMode(ENC1, INPUT);
  pinMode(ENC2, INPUT);
  pinMode(12, OUTPUT); // this is the status flag indicator 
  Serial.begin(9600);

  // initialize the motor object 
  md.init();
  
  // initialize ISR
  Serial.println("Mini Project");
  Serial.println("Ready!");


  /*
  // initialize I2C as the subordinate 
  Wire.begin(SLAVE_ADDRESS);

  // callbacks for I2C
  Wire.onRequest(sendData);
  Wire.onReceive(receiveData);
  Serial.println("Mini Project");
  Serial.println("Ready!");
*/
}

void loop() { // we ,ay want to change the conditional count values in order to give some tolerance 
  currentPosition = encoderValues.read();
  rad = (currentPosition/CPR)*2*3.1415;
  Serial.println(rad);
  if(rad>=6.2831){
    rad = rad - 6.2831;
    currentPosition = currentPosition - CPR;
  }
  else if (rad <= 6.2831){
    rad = rad + 6.2831;
    currentPosition = currentPosition + CPR;
  }
  switch(desiredPos) { // we will use these switch cases to rotate towards the goal in the shortest route

    case 1: // here we are trying to rotate towards 0/2pi
        
    if (rad != 0){   
      speedControl(0);   
    }

    break;

    case 2: // here we are rotating towards pi/2 


    if (rad != PI/2){
      speedControl(PI/2);
    }
    break;

    case 3: // here we will rotate towards pi 

    if (rad != PI) {
      speedControl(PI);
    }
    break;

    case 4: // here we will rotate towards 3pi/2

    if (rad != (3*PI)/2){
      speedControl((3*PI)/2);
    }
    break;
  }
}
void speedControl(double target){
  unsigned long now = millis();
  unsigned long delay1 = now;
  error = (target - rad);
  voltage = Kp*error;
  pwmOut = (voltage / 7.8) * 127;
  if ((abs(pwmOut) > 255) && (pwmOut < 0)){
    pwmOut = -255;
  }
  else if ((abs(pwmOut) > 255) && (pwmOut > 0)) {
    pwmOut = 255;
  }
  else {
    pwmOut = pwmOut;
  }
  md.setM1Speed(pwmOut);

  unsigned long delay2 = now;
  timeChange = delay2-delay1;

  if (timeChange < PERIOD*loopCounts){
    delay(PERIOD - timeChange);
  }
  loopCounts++;
}

/*
 * DAWSON STUFF
void receiveData(int byteCount){
  while(Wire.available()) {
    desiredPos = Wire.read();
    Serial.print("Data recieved: ");
    Serial.println(desiredPos);
  }
}

void sendData(){
  Wire.write(desiredPos);
}
*/
