#include <Sparki.h> // include the sparki library

float pi = 3.1415926535897932384626;

float rX=0;//robot coords
float rY=0;
float rT=pi/2;

float gX=.2159;//goal coords
float gY=.2794;
float gT= pi/2;

float a = 0.05;
float b = 0.9;
float c = 0.01;

float d = 0.0851; //axel_length_m = 0.0851
float r = 0.025;  //wheel_diameter_m = 0.05
float maxWheelSpeed = 0.0285;

void setup() 
{
  sparki.beep();
  float rX=0.3;
  float rY=0.4;
  float rT=pi/2;
}

void loop() {
  
  sparki.clearLCD();
  float t1 = millis();
  //figure out how to drive
  
  float dX = (gX-rX);
  float dY = (gY-rY);
  
  sparki.print(dX);
  sparki.print(" ");
  
  sparki.print(dY);
  sparki.print(" ");
  
  float rho = sqrt(pow(dX, 2) + pow(dY, 2)); //distance to point
  sparki.println(rho);

  float at2 = atan2(dY,dX);//returns bearing

  float aFromX = pi/2 - at2;
   
  float dT = aFromX - rT; //how far we have to turn
  sparki.print("dT ");
  sparki.println(dT*180/pi);
  
  
  float nu = gT - rT;
  //sparki.println(nu);
  
  
  float x_rp = a*rho; //our desired movement speed
  float theta_rp = b*dT + c*nu; //our desired rotation speed
  
  //sparki.println("speeds:");
  
  //sparki.println(2*x_rp);
  //sparki.println(d*theta_rp);
  
  float leftWheelSpeed = ((2*x_rp) - (theta_rp*d)) / (2*r); //max 0.0285
  float rightWheelSpeed = ((2*x_rp) + (theta_rp*d)) / (2*r); //max 0.0285

  if(rho < 0.02){
    sparki.println(nu);
    leftWheelSpeed = -nu;
    rightWheelSpeed = nu;
    if(abs(nu) < (5 * pi/180)){
      leftWheelSpeed = 0;
      rightWheelSpeed = 0;
    }
  }
  
  float scaleFactor = max(abs(leftWheelSpeed), abs(rightWheelSpeed));
  
  leftWheelSpeed = leftWheelSpeed * (.0285/scaleFactor);
  rightWheelSpeed = rightWheelSpeed * (.0285/scaleFactor);
  
  sparki.println((leftWheelSpeed/maxWheelSpeed)*100);
  sparki.println((rightWheelSpeed/maxWheelSpeed)*100);
  
  //rotate while moving towards the point
  //eventually, get on the correct trajectory. keep traveling forward,
  //eventually get to the objective point. then rotate to your desired
  //orientation

  if(leftWheelSpeed > 0)
    {
    sparki.motorRotate(MOTOR_LEFT, DIR_CCW, (leftWheelSpeed/maxWheelSpeed)*100); // rotate(which motor, which direction, percent)  
  }
  else{
    sparki.motorRotate(MOTOR_LEFT, DIR_CW, abs((leftWheelSpeed/maxWheelSpeed)*100)); // rotate(which motor, which direction, percent)  
  
    }
  if(rightWheelSpeed > 0)
    {
     sparki.motorRotate(MOTOR_RIGHT, DIR_CW, (rightWheelSpeed/maxWheelSpeed)*100);  
    }
  else{
    sparki.motorRotate(MOTOR_RIGHT, DIR_CCW, abs((rightWheelSpeed/maxWheelSpeed)*100));  
  }
 
  delay(100);
  
  float t2 = millis();
  float totalTime = (t2-t1)/1000;
  
  float Xdot = leftWheelSpeed/2.0 + rightWheelSpeed/2.0;
  float Thetadot = rightWheelSpeed/d-leftWheelSpeed/d;
  
  rX = rX + sin(rT) * Xdot * totalTime;
  rY = rY + cos(rT) * Xdot * totalTime;
  rT = rT + Thetadot * totalTime;

  if (rT>(2*pi)){rT = rT-(2*pi);}
  if (rT<0){rT = rT+(2*pi);}
  
  sparki.print("x: ");
  sparki.print(rX);
  
  sparki.print("  y: ");
  sparki.print(rY);
  
  sparki.print("  T: ");
  sparki.println(dT * (180/pi));

  sparki.print("spin:");
  sparki.print(Thetadot);

  sparki.print(" sp:");
  sparki.print(Xdot);

  sparki.updateLCD();
  
  //DLC 170
}
