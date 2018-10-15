#define NO_LCD 
#include <Sparki.h> // include the sparki library 
int angle=-30; 
void setup() {
  sparki.beep();  
} 

//////////////////////////////////////////////////////
//////////////////////////////////////////////////////
//Our sparki has a problem with connection//
//Professor Nicklous also knows about this//
//////////////////////////////////////////////////////
//////////////////////////////////////////////////////

void loop() { 
 //sparki.servo(SERVO_CENTER); 
 sparki.servo(angle);
 angle=angle+1; 
 if(angle>30){ 
   angle=-30; Serial.println(); 
 } 
int cm = sparki.ping(); // measures the distance with Sparki's eyes 
  Serial.print(cm); 
  Serial.print(" "); 
} 
