/*
    AUTHOR       : Thomas Green
    PROJECT      : Automotive Driving Robot GDP - Group 11, University of Southampton
    DATE STARTED : 09/02/2022

    
    NOTES:
    This code is used to read the position of the prototype pedal using the built-in hall effect
    sensors. The pedal has two hall effect sensors for redundancy and so a different read out is given for
    each of the sensors.

    MAIN HALL EFFECT SENSOR PINOUT:
    POWER(GREEN) --> 5V
    GROUND(WHITE) --> GND
    SENSE(GREY) --> A1

    BACKUP HALL EFFECT SENSRO PINOUT:
    POWER(YELLOW) --> 5V
    GROUND(BLUE) --> GND
    SENSE(RED) --> A2

*/

#define HALL_MAIN A1
#define HALL_BACKUP A2
#define VOLTAGE 5
#define MAX_MAIN_VOLTAGE 3.24       //These values need to be found experimentally, they corrospond to the
#define MIN_MAIN_VOLTAGE 0.79       //senseMain and senseBackup voltage when the pedal is fully depressed(MAX)
#define MAX_BACKUP_VOLTAGE 4.12     //and fully released(MIN).
#define MIN_BACKUP_VOLTAGE 1.57

float senseMain, senseBackup, voltMain, voltBackup, percMain, percBackup;

void setup() {
  Serial.begin(57600);
  
  pinMode(HALL_MAIN, OUTPUT);
  pinMode(HALL_BACKUP, OUTPUT);
}

void loop() {

  //Read the sense pin values and store as floats. These values will be between 0 - 1023
  senseMain = analogRead(HALL_MAIN);
  senseBackup = analogRead(HALL_BACKUP);
   
  // Convert the analogue reading (0 - 1023) to a voltage (0 - 5V):
  voltMain = senseMain * (VOLTAGE/1024.0);
  voltBackup = senseBackup * (VOLTAGE/1024.0);

  //Express the hall effect reading as a throttle percentage
  percMain = (voltMain - MIN_MAIN_VOLTAGE)/(MAX_MAIN_VOLTAGE - MIN_MAIN_VOLTAGE);
  percBackup = (voltBackup - MIN_BACKUP_VOLTAGE)/(MAX_BACKUP_VOLTAGE - MIN_BACKUP_VOLTAGE);
  
  // Printing the postitions for main and backup sensor
  Serial.print("Main Sensor  Analogue Reading:  ");
  Serial.print(senseMain);
  Serial.print("Voltage Reading:  ");
  Serial.print(voltMain);
  Serial.print("Throttle Percentage:  ");
  Serial.print(percMain);
  Serial.println();

  Serial.print("Backup Sensor  Analogue Reading:  ");
  Serial.print(senseBackup);
  Serial.print("Voltage Reading:  ");
  Serial.print(voltBackup);
  Serial.print("Throttle Percentage:  ");
  Serial.print(percBackup);
  Serial.println();

}
