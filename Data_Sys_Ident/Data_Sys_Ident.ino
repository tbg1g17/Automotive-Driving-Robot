/* (04/02/2022)
 * 
 * This code is to collect data from inputs and outputs of our robot and store the data in a csv file.
 * 
 * The code is to be used with the open-loop code created by Tom and the code for creating random PWM signal
 * where open-loop operation of the entire robot is held so that inputs and outputs to the motor are 
 * recorded and used with MATLAB's system-identification toolbox, hence finding the (approx.) TF of our 
 * entire system, relating the input voltage to the pedal's position (i.e., system output), or relating motor 
 * position to pedal position.
 * 
 */


long pwm, V, start;
int dir;


void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(9600);
  randomSeed(24);
  dir = 1;

  start = millis();

}

void loop() {
  // put your main code here, to run repeatedly:

  if (millis() - start >= 500){
    pwm = random(0, 256) * dir;
//    dir = !dir; // change direction
    dir *= -1;
    start = millis();
  }

  Serial.print(millis());
  Serial.print(", ");
  Serial.println(pwm);

  if (millis() >= 10000){
    delay(10000);
    
  }
}
