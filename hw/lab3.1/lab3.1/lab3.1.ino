/**
this is built upon Corrells lab2 code

**/

//#include <QueueArray.h>
#include <Sparki.h>       // include the sparki library

float maxspeed=0.0285;    // [m/s] speed of the robot that you measured
float alength=0.0851;     // [m] axle length  
float phildotr=0, phirdotr=0; // wheel speeds that you sent to the motors
float Xrdot, Thetardot;

const float blockSizeX = 0.148;
const float blockSizeY = 0.1;

float Xi=blockSizeX/2, Yi=blockSizeY/2, Thetai=0;
// note these coords are only valid when starting
// in the lower left corner of the map.

float eX, eY;



//sample map
int blockedSquares[4][4] = {{1,1,1,1},{1,0,0,0},{1,1,1,1},{1,1,1,1}};

//empty map
//int blockedSquares[4][4] = {{1,1,1,1},
//                            {1,1,1,1},
//                            {1,1,1,1},
//                            {1,1,1,1}};


void setup()
{
  Serial.begin(9600);
  
}

void loop() {
 long int time_start = millis();
 int threshold = 700;
 int lineLeft   = sparki.lineLeft();   // measure the left IR sensor
 int lineCenter = sparki.lineCenter(); // measure the center IR sensor
 int lineRight  = sparki.lineRight();  // measure the right IR sensor

 if ( lineCenter < threshold ) // if line is below left line sensor
 {  
   sparki.moveForward(); // move forward
   phildotr=maxspeed;
   phirdotr=maxspeed;
 }
 else{
   if ( lineLeft < threshold ) // if line is below left line sensor
   {  
     sparki.moveLeft(); // turn left
     phildotr=-maxspeed;
     phirdotr=maxspeed;
   }
 
   if ( lineRight < threshold ) // if line is below right line sensor
   {  
     sparki.moveRight(); // turn right
     phildotr=maxspeed;
     phirdotr=-maxspeed;
   }
 }


 sparki.clearLCD(); // wipe the screen
 
 sparki.print(Xi);
 sparki.print("/");
 sparki.print(Yi);
 sparki.print("/");
 sparki.print(Thetai);
 sparki.println();
  
 // perform odometry
 Xrdot=phildotr/2.0+phirdotr/2.0;
 Thetardot=phirdotr/alength-phildotr/alength;
 
 Xi=Xi+cos(Thetai)*Xrdot*0.2;
 Yi=Yi+sin(Thetai)*Xrdot*0.2;
 Thetai=Thetai+Thetardot*0.2;

 int blockIndex = indexFromCoords(Xi, Yi);
  
 sparki.print("Index: ");
 sparki.println(blockIndex);


 eX = coordsFromIndex(blockIndex, 'x');
 eY = coordsFromIndex(blockIndex, 'y');
 
 sparki.println("aprox coords: ");
 sparki.print(eX);
 sparki.print("/");
 sparki.println(eY);

 int distance[16];
 for (int i=0;i<16;i++){
  distance[i]=999;
  }
 myDijk(blockIndex, 16, distance);
 
 sparki.println("distance to 15: ");
 sparki.println(distance[15]);
 
 sparki.clearLCD();

 for(int y=3; y>=0; y--){
  for(int x=0; x<4; x++){
    if ((x==eX)&(y==eY)){
      sparki.print("S");
      }
    else{
      sparki.print(blockedSquares[y][x]);
      }
    }
  sparki.println();
  }

  Serial.println("printing Distances:"); 
  for(int i=0; i<16;i++){
    Serial.print(distance[i]);
    if (i%4 ==0){Serial.println();}
    }
  Serial.println();
 //sparki.drawChar(xCenter, blockY*2, 'S'); // draw a character char at coordinates xPixel, yLine (8 lines on the LCD) 



 
 sparki.updateLCD(); // display all of the information written to the screen
 
 
 while(millis()<time_start+200); // wait until 100ms have elapsed
}




//square index numbers are:
// 12,13,14,15
// 8, 9, 10,11
// 4, 5, 6, 7
// 0, 1, 2, 3

int indexFromCoords(float xPos, float yPos)
{
  int xBlock = floor(xPos/blockSizeX); // ranges from 0-3
  int yBlock = floor(yPos/blockSizeY); // ranges from 0-3
  int blockIndex = 4*yBlock + xBlock; // ranges from 0-15
  return blockIndex;
}





float coordsFromIndex(int i, char xy)//since you cant return two values, we are going to carry a flag of which value to return.
{
  int xI = i%4;
  int yI = i/4;//map coords

  float xPos = (xI+0.5)*blockSizeX;
  float yPos = (yI+0.5)*blockSizeY;//physical coords in cm
  
  if (xy == 'x'){return xI;}
  if (xy == 'y'){return yI;}
  
}


int easyCost(int startIndex, int finishIndex){
  int sX = startIndex%4;
  int sY = startIndex/4;

  int fX = finishIndex%4;
  int fY = finishIndex/4;

  if ((blockedSquares[sY][sX]==1) && (blockedSquares[fY][fX])==1){//check for blocks on start and end
    return (abs(sX-fX) + abs(sY-fY));
    }
  else{
    return 999;
    }  
  }





//void dij(int n,int v,int cost[10][10],int dist[])
//{
// int i,u,count,w,flag[10],min;
// for(i=1;i<=n;i++)
//  flag[i]=0,dist[i]=cost[v][i];
// count=2;
// while(count<=n)
// {
//  min=99;
//  for(w=1;w<=n;w++)
//   if(dist[w]<min && !flag[w])
//    min=dist[w],u=w;
//  flag[u]=1;
//  count++;
//  for(w=1;w<=n;w++)
//   if((dist[u]+cost[u][w]<dist[w]) && !flag[w])
//    dist[w]=dist[u]+cost[u][w];
// }
//}


void myDijk(int startIndex, int n, int dist[]){
  int i,u,count,w,flag[n], min;
  
  for (i=1;i<=n;i++){
    flag[i]=0;
    dist[i]= easyCost(startIndex, i);
    
  }
  
  count=2;
  while (count<=n){
    min=99;
    
    for (w=1;w<=n;w++){
      if (dist[w]<min && !flag[w]){
        min=dist[w];
        u=w;
      }
      flag[u]=1;
      count++;
      
      for(w=1;w<=n;w++){
        if ((dist[u]+ easyCost(u,w)< dist[w]) && !flag[w]){
          dist[w] = dist[u]+ easyCost(u,w);
        }
      }
    }
  }
  Serial.println(dist[5]);
}

























//int BFS(int startIndex, int finishIndex)
//{
//  QueueArray <int> queue;
//  
//  int checked[16];
//  for (int a = 0; a < 16; a++) checked[a] = 0;
//  
//  int parents[16];
//  for (int a = 0; a < 16; a++) parents[a] = 0;
//  
//  queue.enqueue(startIndex);
//  checked[startIndex]=1;
//  
//  while (!queue.isEmpty()){
//    int cur = queue.dequeue();
//    
////    sparki.clearLCD();
////    sparki.println(cur);
//    
//    if (cur == finishIndex){break;}
//    if (blockedSquares[cur/4][cur%4] != 0){
//        
//      int c1 = cur+1;
//      int c2 = cur-1;
//      int c3 = cur+4;
//      int c4 = cur-4;
//      
//      if((c1>=0) && (c1<=15) && (checked[c1]==0)){
//  //    sparki.println(c1);
//      Serial.print(c1);
//        checked[c1] = 1;
//        parents[c1] = cur;
//        queue.enqueue(c1);
//        }
//      if((c2>=0) && (c2<=15) && (checked[c2]==0)){
//  //    sparki.println(c2);
//      Serial.print(c2);
//        checked[c2] = 1;
//        parents[c2] = cur;
//        queue.enqueue(c2);
//        }
//      if((c3>=0) && (c3<=15) && (checked[c3]==0)){
//  //    sparki.println(c3);
//      Serial.print(c3);
//        checked[c3] = 1;
//        parents[c3] = cur;
//        queue.enqueue(c3);
//        }
//      if((c4>=0) && (c4<=15) && (checked[c4]==0)){
//  //    sparki.println(c4);
//      Serial.print(c4);
//        checked[c4] = 1;
//        parents[c4] = cur;
//        queue.enqueue(c4);
//        }
//  //    sparki.updateLCD();
//    }
//    }
//
//    int step1 = parents[finishIndex];
//    int stepCount = 1;    
//    while(step1 != startIndex){
//      stepCount += 1;
//      step1 = parents[step1];
//      
//      }
//    return stepCount;
// }
