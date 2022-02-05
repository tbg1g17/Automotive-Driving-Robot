/*  Taken from the <FreqCount.h> library's example
 * 
 *  Library Documentation can be found in below link
 *  http://www.pjrc.com/teensy/td_libs_FreqCount.html
 *
 *  Works best with frequencies ABOVE 1000 Hz
 *  
 *  Freq measured by sending pwm signal from "PIN" to 
 *  pin 5.
 *  
 *  Wiring: A direct cable can be connected from the 
 *  output pin to pin 5.
 *  
 */
 
#include <FreqCount.h>
//#include <PWMFreak.h>

/* measure pwm out freq of "PIN" */

#define PIN 6 /* For Uno: Can only measure freq of 6 */
int val;


void setup() {
  Serial.begin(9600);
  FreqCount.begin(1000); /* time (in ms) within which pulses 
                          are counted - 1000 gives freq directly*/
  val = 200;
//  setPwmFrequency(6, 1024);
}

void loop() {

  analogWrite(PIN, val); // must be (0 < val <255)
  
  if (FreqCount.available()) {
    unsigned long count = FreqCount.read();
    Serial.println(count);
  }
}
