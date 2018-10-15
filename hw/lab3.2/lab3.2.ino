#include <Sparki.h>

const int NUMNODES = 16;
const int LARGE_INT = 100;
const int BLOCK_WEIGHT = 50;

int path[16];

//map
int blockedSquares[4][4] = {{1,1,1,1},
                            {0,0,0,1},
                            {0,1,1,1},
                            {0,0,0,1}};

void dijkstra(int startIndex, int goalIndex);
int minNotChecked(int checked[], int arr[], int size);
int isEmpty(int arr[], int size);
int inBounds(int neighbor, int cur);
int pathWeight(int current, int neighbor);
int isPath(int current, int neighbor);


void setup() 
{
  sparki.beep();
}


void loop() { 

  dijkstra(0,6);
  delay(5000);
}

void dijkstra(int startIndex, int goalIndex)
{

  int cur = startIndex;
  int checked[16] = {0};
  int parents[16] = {0};
  
  int dist[16];
  
  for (int i = 0; i < NUMNODES; i++)
  {
    dist[i] = LARGE_INT;
  }
  
  dist[cur] = 0;

  while (!isEmpty(checked, NUMNODES))
  {
   
    cur = minNotChecked(checked, dist, NUMNODES);
    checked[cur] = 1; 
    
    int neighbors[4] = {cur-1,
                        cur+4,
                        cur+1,
                        cur-4};
                        
    for (int i = 0; i < 4; i++)
    {
      if (inBounds(neighbors[i], cur) && isPath(cur, neighbors[i]))
      {
        int altDist = dist[cur] + 1;
        if (altDist < dist[neighbors[i]])
        {
          dist[neighbors[i]] = altDist;
          parents[neighbors[i]] = cur;
        }
      }
    }

    int curPath = goalIndex;

    int i = 0;
    while (curPath != startIndex)
    {
      path[i] = curPath;
      curPath = parents[curPath];
      i++;
    }

    for (int i = 0; i < NUMNODES; i++)
    {
      Serial.println(path[i]);
    }
  }
}

int minNotChecked(int checked[], int arr[], int size)
{
  //ret index of minimum value 
  int min = 999;
  int minIndex;
  
  for (int i = 0; i < size; i++)
  {
    if ((arr[i] < min) && (!checked[i]))  
    {
      min = arr[i];
      minIndex = i;     
    }
  }
 
  return minIndex;
  
}

int isEmpty(int arr[], int size)
{
  //zero in arr is not checked, one is checked
  for (int i = 0; i < size; i++)
  {
    if (arr[i] == 0)
    {
      return 0;
    }
  }
  return 1;
}

int inBounds(int neighbor, int current)
{
  int currentX = current%4;
  int currentY = current/4;
  
  int neighborX = neighbor%4;
  int neighborY = neighbor/4;

  //valid coords would be (+1,0), (-1,0), (0,+1), (0,-1)
  int diffX = abs(neighborX-currentX);
  int diffY = abs(neighborY-currentY);

  if ((neighbor > 15) || (neighbor < 0))
  {
    return 0;
  }

  if ((diffX == 1) && (diffY == 0))
  {
    return 1;
  }

  else if ((diffX == 0) && (diffY == 1))
  {
    return 1;    
  }

  return 0; 
}

int isPath(int current, int neighbor)
{
  int currentX = current%4;
  int currentY = current/4;
  
  int neighborX = neighbor%4;
  int neighborY = neighbor/4;
  
  if ((blockedSquares[currentX][currentY] == 0) || (blockedSquares[neighborX][neighborY] == 0))
  {
    return 0; 
  }
  return 1;
}

