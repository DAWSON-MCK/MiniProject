
#include "Encoder.h"

// Here we define our pins and constants

#define pwmMod        9 // used to modify the ISR
#define CPR           3200 // counts per revolution 
#define PERIOD        5 // our sampling rate 
#define pi            3.141// pi constant
Encoder encoderValues(2,3);

// Here we define the variables for position and velocity 
float oldPosition = 0;
float newPosition = 0;
float calcPosition;
float angVelocity;

// Here we define the variables 
float constantTime;
float oldTime = 0;
float newTime = 0;
float deltaTime;
float delay1;
float delay2;

// voltage variable and loop counter 
int voltage;
int loopCounter = 1;

void setup() {
  pinMode(4, OUTPUT); // here we set the tri-state
  digitalWrite(4, HIGH);
  
  pinMode(7, OUTPUT); // sets the motor 1 direction to high
  digitalWrite(7, HIGH);
  
  pinMode(pwmMod, OUTPUT); // motor 1 pwm 
  //analogWrite(pwmMod, 255); // sets the voltage all the way high 
  
  pinMode(8, OUTPUT); // motor 2 direction 
  pinMode(10, OUTPUT); // motor 2 pwm
  
  pinMode(12, INPUT); // status flag indicator 

  Serial.begin(250000); // baud rate for the serial monitor
  Serial.println("READY!");
  Serial.println("Time, Voltage, Angular Velocity");

}

void loop() {
  constantTime = millis();

  if (loopCounter == 1) {
    while(millis() < 1000); // wait a second before we ramp up 
    analogWrite(pwmMod, 255); // sets the voltage all the way high 
  }
  
  delay1 = constantTime;
  newTime = constantTime;
  voltage = analogRead(pwmMod);
  newPosition = encoderValues.read();
  calcPosition = (newPosition/CPR)*2*pi;
  angVelocity = ((calcPosition - oldPosition)/(newTime - oldTime))*1000;

  if((oldPosition != newPosition) && (oldTime != newTime)){
    oldPosition = calcPosition;
    oldTime = newTime;
  }

  if (constantTime < 3000){
    Serial.print(newTime);
    Serial.print("\t");
    Serial.print(voltage);
    Serial.print("\t");
    Serial.print(angVelocity);
    Serial.print("\n");
  }
  loopCounter = loopCounter + 1;
  delay2 = constantTime;

  if ((delay2 - delay1) < loopCounter*PERIOD + 1000){
    delay(PERIOD-(delay2-delay1));
  }
}
