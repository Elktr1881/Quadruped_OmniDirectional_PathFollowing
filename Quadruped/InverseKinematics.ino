
// Speed of trajectory based on DelayMoving
#define DelayMoving 5000 //in microsecond (us) 7000

// lenght of Quadrupped in centimeters (cm)
const float Coxa  = 3.00;
const float Femur = 7.00;
const float Tibia = 12.00;
const float math_acc = 0.00000001;

void inverseKinematics(int ftNumb, float kx, float ky, float kz)
{
    float koorX = kx + math_acc;
    float koorY = ky + math_acc;
    float koorZ = kz + math_acc;

    float rangeXY = sqrt(pow(koorY,2)+pow(koorX,2));

    float rangeCoxa = sqrt(pow((rangeXY-Coxa),2)+pow(koorZ,2));

    float angleCoxaR = (atan(koorY/koorX)*180/phi);
    float angleCoxaL = (atan(koorY/koorX)*180/phi);
    if(kx < 0){angleCoxaR = 180 + angleCoxaR;}
    if(kx < 0){angleCoxaL = 180 + angleCoxaL;}

    float triangleFemur = asin(koorZ/rangeCoxa)*180/phi;
    float testFemur = acos((pow(Femur,2)+pow(rangeCoxa,2)-pow(Tibia,2))/(2*Femur*rangeCoxa))*180/phi;
    float angleFemurR = 90-(testFemur-triangleFemur);
    float angleFemurL = 90+(testFemur-triangleFemur);

    float angleTibiaR = (acos((pow(Femur,2)+pow(Tibia,2)-pow(rangeCoxa,2))/(2*Tibia*Femur))*180/phi);
    float angleTibiaL = 180-(acos((pow(Femur,2)+pow(Tibia,2)-pow(rangeCoxa,2))/(2*Tibia*Femur))*180/phi);

    if(ftNumb == frontRight or ftNumb == backRight)
    {posCoxa[ftNumb]=angleCoxaR; posFemur[ftNumb]=angleFemurR; posTibia[ftNumb]=angleTibiaR;}
    else if(ftNumb == frontLeft or ftNumb == backLeft)
    {posCoxa[ftNumb]=angleCoxaL; posFemur[ftNumb]=angleFemurL; posTibia[ftNumb]=angleTibiaL;}

    posCoxa[ftNumb]   = constrain(posCoxa[ftNumb],0,180);
    posFemur[ftNumb]  = constrain(posFemur[ftNumb],0,180);
    posTibia[ftNumb]  = constrain(posTibia[ftNumb],0,180);
    
    coxa[ftNumb].write(posCoxa[ftNumb]);
    femur[ftNumb].write(posFemur[ftNumb]);
    tibia[ftNumb].write(posTibia[ftNumb]);

//    if(ftNumb == 0)
//    { 
//        Serial.print("X:"); Serial.print(kx,2); Serial.print("\t");
//        Serial.print("Y:"); Serial.print(ky,2); Serial.print("\t");
//        Serial.print("Z:"); Serial.print(kz,2); Serial.print("no:"); Serial.println(k);
//        
//        Serial.print("C: "); Serial.print(posCoxa[ftNumb]); Serial.print("\t");
//        Serial.print("F: "); Serial.print(posFemur[ftNumb]); Serial.print("\t");
//        Serial.print("T: "); Serial.print(posTibia[ftNumb]); Serial.print("\n");
//    }
}

void trajectory(float res)
{
    float w[3][4];
    for(int leg=0; leg<4; leg++)
    {
      w[0][leg] = posX[leg]-X[leg];
      w[1][leg] = posY[leg]-Y[leg];
      w[2][leg] = posZ[leg]-Z[leg];
    }
    
    float W = 0.00;
    for(int vec=0; vec<3; vec++)
    {
      for(int leg=0; leg<4; leg++)
      {if(W <= abs(w[vec][leg])){W = abs(w[vec][leg]);}}
    }
    
    float p[3][4];
    for(int vec=0; vec<3; vec++)
    {
      for(int leg=0; leg<4; leg++)
      {p[vec][leg] = w[vec][leg]/W;}
    }

    for(int gap=0; gap<W/res; gap++)
    {
      for(int feetNumb=0; feetNumb<4; feetNumb++)
      {
        inverseKinematics(feetNumb, posX[feetNumb], posY[feetNumb], posZ[feetNumb]);
        posX[feetNumb]-=p[0][feetNumb]*res;
        posY[feetNumb]-=p[1][feetNumb]*res;
        posZ[feetNumb]-=p[2][feetNumb]*res;

        delayMicroseconds(DelayMoving);
      }   
    }
}
