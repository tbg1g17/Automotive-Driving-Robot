#include <stdio.h>
#include <math.h>

#define SPEED 255
#define PWM 3 //PWM pin
#define DIR 2 //Direction pin
#define MAXANG 300 //Maximum angle
#define MINANG 60 //Minimum angle
#define THRESHOLD 5
#define ACW 0
#define CW 255


void setup()
{
  pinMode(PWM, OUTPUT);
  pinMode(DIR, OUTPUT);
  Serial.begin(9600);
}

void loop()
{
  
  double throttlePer;
  while(Serial.available()==0){
    Serial.println(potAngle());
    }
  
  while(Serial.available()>0) {
    Serial.println("Done");
    throttlePer = Serial.parseFloat(SKIP_WHITESPACE);
    Serial.println(throttlePer);
    setThrottle(throttlePer);
  }

}


void setThrottle(double per){

  double pedAng = ((per/100.0)*(MAXANG-MINANG))+MINANG;
  //Serial.println(pedAng);
  setPos(pedAng);
  
}

void setPos(double ang){

  /*If the desired angle is outside possible 
    pedal angles raise error*/
  if(ang<MINANG or ang>MAXANG){
    Serial.println("Error: Angle our of bounds");
    analogWrite(PWM, 0);
  }
  else{
    /*If current angle less than desired angle
      rotate motor CW*/
    while(ang>potAngle()+THRESHOLD){ 
      analogWrite(PWM, SPEED);
      analogWrite(DIR, CW);
    }

    /*If current angle greater than desired angle
      rotate motor ACW*/
    while(ang<potAngle()-THRESHOLD){
      analogWrite(PWM, SPEED);
      analogWrite(DIR, ACW);
    }
    
    /*If the current position is at desired angle 
      (or within threshold) turn off motor*/
    if(potAngle()+THRESHOLD<=ang<=potAngle()-THRESHOLD){
      analogWrite(PWM, 0);
      Serial.println("OFF");
    }
  }
}



double potAngle(){

    double potValue = analogRead(A0);//Getting the current potentiometer value
    double potAngleCart = (245/1023.0)*potValue;
//Convering the pot position to cartesian angle

    return potAngleCart;
}
