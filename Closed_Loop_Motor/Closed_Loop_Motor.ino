/*

    AUTHOR       : Mahmoud GABR
    PROJECT      : GDP - Group 11, University of Southampton
    DATE STARTED : 28/01/2022


    NOTES:
    This code is the first step where the PID controls the position of the motor shaft using the feedback from the
    pot mounted (somehow) to the motor's shaft. There is another a copy of this sketchbook called "Closed_Loop_Pedal"
    where the PID controls the position of the throttle pedal by using the feedback from the pedal's own sensor.


    TO DO:
    Below are further developments that should/could be added to the code.

    1. Add Interrupt Service Routine (ISR) for E-Stop ----------------------------------------------------------> DONE
    2. (Maybe) Change setpoint such that it's % of full throttle (i.e. 0-100)-----------------------------------> DONE
    3. Understand <Oversampling.h> AND <SmoothADC.h> to choose one
    4. Find a way to wire up a pull-up resistor with a button
    5. IMPORTANT: Code is setup and PID tuned such that it controls position of motor pot.
       But setpoint of motor pot is calculated from the user-given % of full-throttle (setpoint)
       Hence, we need to have the TF relating pot position to pedal position (probs through Sys-ID method)

*/



#include <Oversampling.h> /* OR */ #include <SmoothADC.h> /* Oversampling or ADC??
https://dsp.stackexchange.com/questions/63954/trouble-with-understanding-moving-average-filter-as-an-oversampling-filter */
#include <PWMFreak.h>
#include <PID_v1.h>

#define POT_PIN       A0                    //pin reading feedback from pot connected to motor shaft
#define MOTOR_DIR_PIN 8                     //pin controlling direction of motor rotation (digital)
#define MOTOR_PWM     9                     //pin controlling motor speed (pin adjusting freq must not affect delay/millis fn)
#define E_STOP_PIN    2                     //pin for attachInterrupt() to control e_stop button status
#define MAX_PER       100
#define MIN_PER       0
#define THRESH        0.1                   //erro less than THRESH is ignored and set to 0

volatile byte e_stop_state = 1;             //(1) -> button not pressed; (0) -> button pressed, stop robot
byte motor_dir;                             //(0) -> forward           ; (1) -> reverse
int pol;
double perce_set;                           //desired shaft position as % of the 240 deg max pot rotation

double pid_set, pid_in, pid_out;            //pid_setpoint (0-1024), input (0-1024) & output (0-255, changeable if needed)
double Kp, Ki, Kd;                          //PID Gains

PID myPID(&pid_in, &pid_out, &pid_set,
          Kp, Ki, Kd, DIRECT);              //creating PID Object


void setup() {

  Serial.begin(57600);
  Serial.setTimeout(50);

  /*plotting legends */
  Serial.print("SETPOINT:, CURRENT_POSITION:");
  Serial.println();


  pinMode(POT_PIN, INPUT);
  pinMode(MOTOR_DIR_PIN, OUTPUT);
  pinMode(MOTOR_PWM, OUTPUT);
  pinMode(E_STOP_PIN, INPUT_PULLUP);

  /* Kill Switch */
  attachInterrupt(digitalPinToInterrupt(E_STOP_PIN), E_stop, LOW);


  analogReference(INTERNAL);                //INTERNAL to use 1.1 V internal reference, DEFAULT to use 5V Vcc
  setPwmFrequency(MOTOR_PWM, 8);            //set PWM freq to 3906 Hz
  burn_8_readings();                        //burns the first few analog readings which are likely innacurate


  Kp = 0., Ki = 0., Kd = 0.;                //specifying values of PID gains
  myPID.SetMode(AUTOMATIC);                 //turn PID on in Automatic mode
  myPID.SetSampleTime(100);                 //the default is 100 ms, but could reduce it for more

  perce_set = 0;
  pid_set = percToAnalog(perce_set);        //map initial setpoint from 0-100 onto 0-240

}


void loop() {

  
  if (digitalRead(E_STOP_PIN) == 0) {} //if button is pressed, exit entire code (Add Interrupt Routine)
  
  
  /* take input & check it's within range and transform to pid_set */
  if (Serial.available()) {
    perce_set = Serial.parseFloat();

    if (perce_set > MAX_PER or perce_set < MIN_PER) Serial.println("Input out of range (0-100)");

    else {
      Serial.print("Given % input is: ");
      Serial.println(perce_set);
      pid_set = percToAnalog(perce_set);
    }
  }


  /* Using SMA to find pot_reading */
  pid_in = SMA(POT_PIN, 25.0);


  /* Specify direction based on overshoot or undershoot */
  if (pid_in - pid_set >= THRESH) {
    motor_dir = LOW;
    pol = 1;
  }
  else if (pid_in - pid_set <= -THRESH) {
    motor_dir = HIGH;
    pol = -1;
  }
  else { //if error too small, approx it to zero to avoid flutuation
    pid_in = pid_set;
    pol = 0;
  }


  /* Compute PID output and write it to motor */
  myPID.Compute();
  digitalWrite(MOTOR_DIR_PIN, motor_dir);
  analogWrite(MOTOR_PWM, pid_out*pol);


/* Plotting (only when no other Serial values are printed) */
//  Serial.print(pid_set);
//  Serial.print(", ");
//  Serial.print(pid_in);
//  Serial.print(", ");
//  Serial.println();


}





double analogToPerc(double analog_val) { //assumes analog_val is between (0-1024)
  return (analog_val * 100.0) / 1024.0;
}

double percToAnalog(double perc_val) { //assumes perc_val is between (0-100)
  /**************************************************************************************************
     The PID controller will control the angle of th e shaft of the motr between approx. 0 to 20 deg.
     However, it will read input pot values between 0 to 1024 and by default, output is between 0-255
     Since we specify our setpoint as pedal position/angle in %, we must first map it to range
     from 0-1024, so the PID can compare it to the pot reading.
   **************************************************************************************************/
  return (perc_val / 100.0) * 1024.0;
}


double SMA(int reading_pin, double num_counts) {
  double sum = 0;
  
  for (double i = 0; i <= num_counts; i++) {
    sum += analogRead(reading_pin);
    delay(2);
  }
  
  return sum / num_counts;
}

void burn_8_readings(void) {
  // burns the first few analog readings which R likely innacurate
  int i;
  for (i = 0; i < 8; i++) {
    analogRead(POT_PIN);
  }
}

void E_stop(){
  exit(0);
}