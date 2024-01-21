
#define runForward 11
#define runRight   21
#define runLeft    31
#define runBack    41
#define runRotation   51
#define Finished      61

float range;
int Angle;
unsigned int timeState=0, capState=0;
unsigned int stateOftime = 0, posTime = 0;
bool stateOfmark = false;
int stateRoute, modeRoute;
const int routeFollowing[4][6] = {{runLeft, runBack, runBack, Finished, Finished, Finished},
                                  {runRight, runForward, runForward, runLeft, runBack, Finished},
                                  {runForward, runLeft, runForward, runRight, Finished, Finished},
                                  {runBack, runRight, runBack, runRight, runForward, Finished}};
                                  
//const int routeFollowingR1[4][5] = {runLeft, runBack, runBack, Finished};
//const int routeFollowingR2[4][6] = {runRight, runForward, runForward, runLeft, runBack, Finished};
//const int routeFollowingR3[4][5] = {runForward, runLeft, runForward, runRight, Finished};
//const int routeFollowingR4[4][6] = {runBack, runRight, runBack, runRight, runForward, Finished};

void getRange()
{
  received_Data();
  Serial3.println("Yaw: " + String(yaw));
  if(setpoint==0 && yaw>=180){yaw = map(yaw, 180, 360, -180, 0);}
  range = setpoint-yaw;

  if(range>6){range=6;} 
  else if(range<-6){range=-6;}
}

void pathFollowing_Strategy()
{
  timeState = millis() - capState;
  received_Data();
  Serial3.println(String(mark[0]) + String(mark[1]) +  String(mark[2]) + String(mark[3]) + String(mark[4]));

  if((mark[1]==1 or mark[3]==1 or mark[4]==1) and stateOfmark)
  {
    received_Data();
    Serial3.println("Yaw: " + String(yaw));
    delay(1000);
    track = runRotation; 
    track = routeFollowing[modeRoute][stateRoute]; 
    stateOfmark = false;
    capState = millis();
  }
  if(mark[0]==0 and mark[1]==0 and mark[3]==0 and mark[4]==0 and !stateOfmark and timeState > 3000)
  {
    stateRoute++;
    stateOfmark = true;  
    if(stateRoute>5){stateRoute=5;}
  }
    
  switch(track)
  {
    case 1:
      received_Data();
      setpoint = 0;
//      Serial3.println("Setpoint: " + String(setpoint));
      track = routeFollowing[0][0];
    break;
    case 2:
      received_Data();
      setpoint = 0;
//      Serial3.println("Setpoint: " + String(setpoint));
      track = routeFollowing[1][0];
    break;
    case 3:
      received_Data();
      setpoint = 0;
//      Serial3.println("Setpoint: " + String(setpoint));
      track = routeFollowing[2][0];
    break;
    case 4:
      received_Data();
      setpoint = 0;
//      Serial3.println("Setpoint: " + String(setpoint));
      track = routeFollowing[3][0];
    break;
    
    case runForward:
      getRange();
      movePoint(10, 3, 6, 1,(range/6)+0.35, 96); // maju
      Move(0.5);
    break;
    
    case runBack:
      getRange();
      movePoint(10, 3, 6, 1,(range/6)+0.3, 273); // belakang
      Move(0.5);
    break;
    
    case runRight:
      getRange();
      movePoint(10, 3, 6, 1,(range/6)+0.2, 180); // kanan
      Move(0.5);
    break;
    
    case runLeft:
      getRange();
      movePoint(10, 3, 6, 1,(range/6)+0.2, 0); // kiri
      Move(0.5);
    break; 

    case runRotation:
      getRange();
      movePoint(10, 0, 6, 1,(range/6)-0.4, 0); 
      Move(0.5);
      if(abs(range)<=1){track = routeFollowing[modeRoute][stateRoute];}
    break;

    case Finished:
      standBy();
      received_Data();
      Serial3.println("Yaw: " + String(yaw));
      track = 0;
    break;
  }
}
