/* Introuction Parameter of movePoint
 * posAngle : 90 (Forward), 0 (Left), 180 (Right), 270 (Back)
 * distance : end effector to second servo
 * gap : step lenght
 * distanceZ : height robot from ground
 * gapz : femur length
 * rotation : CCW(+), CW(-)
 */
void movePoint(float distance, float gap, float distanceZ, float gapZ, float rotation, float posAngle) 
{
   float flagAngle1, flagAngle2;
   flagAngle1 = 135-posAngle;
   flagAngle2 = 225-posAngle;
   if(flagAngle1 < 0){flagAngle1 = 360+flagAngle1;}
   if(flagAngle2 < 0){flagAngle2 = 360+flagAngle2;}

//   Serial.print("Ang1: "); Serial.println(flagAngle1);
//   Serial.print("Ang2: "); Serial.println(flagAngle2);

   if(flagAngle2 <= 180)
   {
       endX[0] = startX[3] = gap*cos(flagAngle2*phi/180);
       endY[0] = startY[3] = distance-(gap*sin(flagAngle2*phi/180));
       startX[0] = endX[3] = 0;
       startY[0] = endY[3] = distance;
   }
   else if(flagAngle2 > 180)
   {
       flagAngle2 -= 180;
       endX[0] = startX[3] = 0;
       endY[0] = startY[3] = distance;
       startX[0] = endX[3] = gap*cos(flagAngle2*phi/180);
       startY[0] = endY[3] = distance-(gap*sin(flagAngle2*phi/180));
   }

   //Reverse
   if(flagAngle1 <= 180)
   {
       endX[2] = startX[1] = 0;
       endY[2] = startY[1] = distance;
       startX[2] = endX[1] = gap*cos(flagAngle1*phi/180);
       startY[2] = endY[1] = distance-(gap*sin(flagAngle1*phi/180));
   }
   else if(flagAngle1 > 180)
   {
       flagAngle1 -= 180;
       endX[2] = startX[1] = gap*cos(flagAngle1*phi/180);
       endY[2] = startY[1] = distance-(gap*sin(flagAngle1*phi/180));
       startX[2] = endX[1] = 0;
       startY[2] = endY[1] = distance;
   }

   startX[0] += rotation/2;
   startX[1] -= rotation/2;
   startX[2] -= rotation/2;
   startX[3] += rotation/2;
   
   endX[0] -= rotation/2;
   endX[1] += rotation/2;
   endX[2] += rotation/2;
   endX[3] -= rotation/2;

   startZ = distanceZ;
   endZ = distanceZ-gapZ;
   
   for(int leg=0; leg<4; leg++)
   {  
//      Serial.print("startX: "); Serial.println(startX[leg]);
//      Serial.print("startY: "); Serial.println(startY[leg]);
//      Serial.print("endX  : "); Serial.println(endX[leg]);
//      Serial.print("endY  : "); Serial.println(endY[leg]);
//      Serial.println();
   }
}

// Move based resolution is how many point of polinomial move
void Move(float resolution)
{
  for(int leg=0; leg<4; leg++)
  {X[leg]=endX[leg]; Y[leg]=endY[leg];}
  Z[0]=Z[3]=startZ;
  Z[2]=Z[1]=endZ;
  trajectory(resolution);
//  delay(2000);
  
  Z[2]=Z[1]=startZ;
  trajectory(resolution);
//  delay(2000);

  for(int leg=0; leg<4; leg++)
  {X[leg]=startX[leg]; Y[leg]=startY[leg];}
  Z[0]=Z[3]=endZ;
  Z[2]=Z[1]=startZ;
  trajectory(resolution);
//  delay(2000);

  Z[0]=Z[3]=startZ;
  trajectory(resolution);
//  delay(2000);
}

void standBy()
{
  inverseKinematics(0,0,11,6);
  inverseKinematics(1,0,11,6);
  inverseKinematics(2,0,11,6);
  inverseKinematics(3,0,11,6);
}
