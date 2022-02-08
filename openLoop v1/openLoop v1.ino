#include <Arduino.h>
#include <stdio.h>
#include <math.h>

#define SPEED 255
#define PWM 3 //PWM pin
#define DIR 2 //Direction pin
#define MAXANG 200 //Maximum angle
#define MINANG 18 //Minimum angle
#define THRESHOLD 5
#define ACW 0
#define CW 255

void setup()
{
  Serial.begin(9600);
  delay(500);
  pinMode(PWM, OUTPUT);
  pinMode(DIR, OUTPUT);
}

void loop()
{
  setPos(80);
  setPos(-80);
  /*
  while(1){
    analogWrite(PWM, 0);
    Serial.println(potAngle());
  }
  */
  
}



void setPos(double ang){

  double tDelay = 1000*abs(ang)*0.011*(100.0/SPEED); //*1000 to convert to seconds, 0.011 is the time per degree in seconds, 100/SPEED to normalise the time for speeds.
  
  if(ang>0){
      analogWrite(PWM, SPEED);
      analogWrite(DIR, CW);
      Serial.println(potAngle());
      delay(tDelay);
      analogWrite(PWM, 0);
  }
  else{
      analogWrite(PWM, SPEED);
      analogWrite(DIR, ACW);
      Serial.println(potAngle());
      delay(tDelay);
      analogWrite(PWM, 0);
  }
  delay(30);
}

double potAngle(){

    double potValue = analogRead(A0); //Getting the current potentiometer value
    double potAngle360 = (270/1023.0)*potValue; //Convering the pot position to cartesian angle

    //Serial.println(potAngle360); //Printing pot angle to serial monitor

    return potAngle360;
}
