#include <Sparki.h>
//#define START 0
//#define MOVE 1
//int state = START;

int interval = 1000;
int cm = 0;
int loopCount = 0;
float maxspeed=0.0285;    // [m/s] speed of the robot that you measured
float alength=0.0851;     // [m] axle length  
float phildotr=0, phirdotr=0; // wheel speeds that you sent to the motors
float Xi=0, Yi=0, Thetai=0;
float Xrdot, Thetardot;

int arrayCounter = 0; //integer to count through commArray
String inputString; // Empty String
boolean returnFlag; //flag to check for carriage return
boolean sentOK = false;
char commArray [20]; //array to store communication

void setup() {
  sparki.servo(SERVO_CENTER);
  Serial1.begin(9600);
  instruction();
}

void commRead(String info) {
  while(Serial.available()){
    int inByte = Serial1.read();

    if ((char)inByte == 'n') // end of line, carriage return
    {
      returnFlag = true;
      arrayCounter = 0;
    }
    else
    {
      if(inByte == 32) //if it's a blank space
      {
        arrayCounter ++; //increment array counter to store in new array space
      }
      else
      {
      //add the character to the arrayCounter space in commArray
      commArray[arrayCounter] += (char)inByte; 
      }
    }
  }
  //debugging/checking to make sure it works
  for(int i = 0; i <= 20; i++)
  {
    Serial1.println(commArray[i]);
  }
}

void updateSensorCM(int angle) {
  String scan = "scan";
  String XiS = (String)" "+ Xi;
  String YiS = (String) XiS + " " + Yi;
  String ThetaiS = (String) YiS +" "+ Thetai;
  String angleS = (String) ThetaiS + " " + angle;
  String pingS = (String) angleS + " " + sparki.ping() + " ";
  String info = "S " + scan + pingS + "E";
  
  sparki.clearLCD();

  //show on sparki
  sparki.print(info);
  //display on serial monitor
  Serial1.println(info);

  commRead(info);
  
  sparki.updateLCD();
  Serial1.println(inputString);  //this code is for debugging  

  delay(500);
}

int scanDir() {
  // those angle ranges are the best for sparki spin his head, i.e. 180 degree
  for (int angle = -85; angle < 81; angle = angle + 10){
    sparki.servo(angle);
    updateSensorCM(angle);
    //updateSensorRead();
    if (angle == 80){
      sparki.servo(SERVO_CENTER);
    }
  }
}

void instruction(){
  while(sentOK == false){
    while(Serial1.available()){
      if(returnFlag){
        inputString = "";
        returnFlag = false;
      }
      //talking to sparki from Serial monitor
      int inByte = Serial1.read();
      Serial.print((char)inByte);
  
      if ((char)inByte == 'n') // n is carriage character
      {
        returnFlag = true;
      }
      else
      {
        Serial1.println("AHHHHHHHH");
        inputString += (char)inByte; //add the character from bluetooth to string
      }
    }

    if ((inputString == "OK" || inputString == "ok" || inputString == "Ok" || inputString == "oK") && !returnFlag)
    {
      Serial1.println("I start moving");
      delay(500);
      sentOK = true; //exit the OK while loop
    }
     //end code to check for OK communication
    else //send instructions once a second
    {
      Serial1.println("Type ok and I start moving!");
      delay(2000);
    }
  }
}

void loop() {
  
  if(sentOK == true){
  
    sparki.moveForward(10);
    Xrdot = phildotr / 2.0 + phirdotr / 2.0;
    Thetardot = phirdotr / alength-phildotr /alength;
    Xi = Xi + cos(Thetai) * Xrdot * 0.1;
    Yi = (Yi + sin(Thetai) * Xrdot * 0.1) + 10;
    Thetai = Thetai + Thetardot * 0.1;
    scanDir();
    delay(interval);
  
  }
}
