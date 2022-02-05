/* 
 *  NOTES:
 *  
 *  This code is the second/final step towards controlling the position of the throttle pedal to reach the % setpoint 
 *  specified by the user and hence the to reby using the feedback from the pedal's own sensor.
 *  
 *  
 *  
 *  TO DO:
 *  
 *  Below are further developments that should/could be added to the code (THOSE ARE GENERIC FOR BOTH Closed-Loop-Codes).
 *  
 *  1. Add Interrupt Service Routine (ISR) for E-Stop
 *  2. (Maybe) Change setpoint such that it's % of full throttle (i.e. 0-100)
 *  3. Understand <Oversampling.h> AND <SmoothADC.h> to choose one
 *  4. Find a way to wire up a pull-up resistor with a button
 *  5. IMPORTANT: Code is setup and PID tuned such that it controls position of motor pot. 
 *     But setpoint of motor pot is calculated from the user-given % of full-throttle (setpoint)
 *     Hence, we need to have the TF relating pot position to pedal position (probs through Sys-ID method)
 *     
 */




#include <Oversampling.h> /* OR */ #include <SmoothADC.h> /* Oversampling or ADC??
https://dsp.stackexchange.com/questions/63954/trouble-with-understanding-moving-average-filter-as-an-oversampling-filter */
#include <PWMFreak.h>
#include <PID_v1.h>

#define POT_IN A0                       //pin reading feedback from pot connected to motor shaft
#define MOTOR_DIR 8                     //pin controlling direction of motor rotation (digital)
#define MOTOR_PWM 9                     //pin controlling motor speed (pin adjusting freq must not affect delay/millis fn)
#define E_STOP_PIN 2                    //pin reading e_stop button status

volatile byte   e_stop_state;           //(1) -> button not pressed; (0) -> button pressed, stop robot
byte   motor_dir;                       //(1) -> forward           ; (0) -> reverse
double desir_perc;                      //desired shaft angle %

double pid_set, pid_in, pid_out;        //pid_setpoint (0-1024), input (0-1024) & output (0-255, changeable if needed)
double Kp, Ki, Kd;                      //PID Gains
double prev_pid_out;                    //stores previous PID computed output

void burn_8_readings(void);

PID myPID(&pid_in, &pid_out, &pid_set, Kp, Ki, Kd, DIRECT); //creating PID Object


void setup() {

  Serial.begin(9600);
  Serial.setTimeout(50);
  
  pinMode(POT_IN, OUTPUT);
  pinMode(MOTOR_DIR, OUTPUT);
  pinMode(MOTOR_PWM, OUTPUT);
  pinMode(E_STOP_IN, INPUT_PULLUP);

  e_stop_stat = 1;

  /* Adjusting Frequencies and Sampling Rates */
  analogReference(INTERNAL1V1);      //use the 1.1 V internal reference (Change to INTERNAL when using UNO)
  setPwmFrequency(MOTOR_PWM, 8);     //set PWM freq to 3906 Hz
  burn_8_readings();                 // burns the first few analog readings which are likely innacurate

  Kp=0., Ki=0., Kd=0.;               //specifying values of PID gains
  myPID.SetMode(AUTOMATIC);          //turn PID on in Automatic mode
  myPID.SetSampleTime(100);          //the default is 100 ms, but could reduce it for more
                                     //frequent output calculations hence (maybe) better performance
  

  /**************************************************************************************************
   * The PID controller will control the angle of th e shaft of the motr between approx. 0 to 20 deg.
   * However, it will read input pot values between 0 to 1024 and by default, output is between 0-255
   * Since we specify our setpoint as pedal position/angle in degrees, we must first map it to range
   * from 0-1024, so the PID can compare it to the pot reading.
   **************************************************************************************************/
  deg_setpoint = 0;
  pid_set = (deg_setpoint*1024.)/20; //map user-specified setpoint from 0-20 onto 0-1024

} /* END OF SETUP() */



void loop() {


    if (Serial.available()){
    desir_perc = Serial.parseFloat();
    Serial.println(deg_setpoint);
  }

  /* Kill Switch */
//  if (digitalRead(E_STOP_PIN) == 0) {exit(0);} //if button is pressed, exit entire code (Add Interrupt Routine)


   /* DO I NEED TO USE OVERSAMPLING OR MOVING AVERAGE?? OR IS ANALOGREFERENCE() ENOUGH? */
   pid_in = analogRead(POT_IN);
   myPID.Compute();
   

  /**************************************************************************************************
    PID doesn't know that it needs to change direction of rotation, only that it needs to increase or 
    decrease output if it overshoots or undershoots respectively.
    So, we setup so that if computed output is greater than previous (undershoot), motor direction 
    remains the same. Otherwise (overshoot), change direction of rotation.
   **************************************************************************************************/
   if (prev_pid_out > pid_out){
    motor_dir = 0;
   }
   else if (prev_pid_out < pid_out){
    motor_dir = 1;
   }

   digitalWrite(MOTOR_DIR, motor_dir);
   analogWrite(MOTOR_PWM, pid_out);
//   myPID.SetTunings(0., 0., 0.); //allows mid-operation changing of initial PID gains set

  prev_pid_out = pid_out;

}


void analogToPerc(int analog_val){
  
}

void burn_8_readings(void){
  // burns the first few analog readings which R likely innacurate
  int i;
  for (i = 0; i < 8; i++){analogRead(POT_IN);}
}
