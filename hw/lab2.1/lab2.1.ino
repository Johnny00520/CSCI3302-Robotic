/*******************************************
 Sparki Line-following example
 
 Threshold is the value that helps you 
 determine what's black and white. Sparki's 
 infrared reflectance sensors indicate white 
 as close to 900, and black as around 200.
 This example uses a threshold of 700 for 
 the example, but if you have a narrow line, 
 or perhaps a lighter black, you may need to 
 adjust.
********************************************/

//#include <Sparki.h>
#include <Arduino.h>

float x = 0;
float y = 0;
float z = 0;
#define movingLeft 1
#define movingRight 2
#define movingForward 3
int state = 0;
double pi = 3.1415926535;

void setup() 
{
  sparki.beep();
  int state = 0;
}

void loop() {
  int threshold = 500;
  
  int lineLeft   = Arduino.lineLeft();   // measure the left IR sensor
  int lineCenter = sparki.lineCenter(); // measure the center IR sensor
  int lineRight  = sparki.lineRight();  // measure the right IR sensor
  int takeRound = 2;
  double last = millis(); //get time at start of loop
  double timing = millis(); // get timing
  
  if ( lineCenter < threshold ) // if line is below left line sensor
  {  
    sparki.moveForward(); // move forward
    //state = 3
    state = movingForward;
  }
  else{
    if ( lineLeft < threshold ) // if line is below left line sensor
    { 
      sparki.moveLeft();
      //state = 1
      state = movingLeft;
    }
  
    if ( lineRight < threshold ) // if line is below right line sensor
    {  
      sparki.moveRight();
      //state = 2
      state = movingRight;
    }

    if ( lineCenter < threshold and lineLeft < threshold and lineRight < threshold)
    {
      sparki.moveStop(); // for the first round
    }

//    if ( lineCenter < threshold and lineLeft < threshold and lineRight < threshold and takeRound)
//    {
//      sparki.moveStop(); // for the first round
//      sparki.print("One circle is: ");
//      sparki.println(timing);
//    }
  }
  
  sparki.clearLCD(); // wipe the screen
  
  sparki.print("Line Left: "); // show left line sensor on screen
  sparki.println(lineLeft);
  
  sparki.print("Line Center: "); // show center line sensor on screen
  sparki.println(lineCenter);
  
  sparki.print("Line Right: "); // show right line sensor on screen
  sparki.println(lineRight);

  //firstTime = millis()
  //sparki.moveForward(30) // 0.3 meters
  //secondTime = millis()
  //sparki.println(secondTime - firstTime)
  //0.0278 in meter/second
  // we need to convert the unit
  sparki.println(.0278 * (millis() / 1000));
  
  sparki.print("(x,y,z): ");
  sparki.println(x);
  sparki.println(y);
  sparki.println(z);

//  sparki.print("State: ");
//  sparki.println(state);
  sparki.print("One circle is: ");
  sparki.println(timing);
  
  sparki.updateLCD(); // display all of the information written to the screen
  delay(100); // wait 0.1 seconds
  
  
  double now = millis();
  
  switch (state){
    case 0:{break;}
    case movingLeft:{
      //direction angle on the left
      z = z - (90 / 2.2) * ((now - last) / 1000);// turn left
      break;
    }
    case movingRight:{
      //direction angle on the right
      z = z + (90 / 2.2) * ((now - last) / 1000);// turn right  
      break;
    }
    case movingForward:
    {
      double dLocal = ((now - last) / 1000) * 0.0278;// sec* m/sec gives meters
      x = x + dLocal * cos( z * (pi / 180));
      y = y + dLocal * sin( z * (pi / 180));
      break;}
    }
}
