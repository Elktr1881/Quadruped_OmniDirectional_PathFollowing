#include <Servo.h>
#define phi  3.14159265

/* Introduction Feet Number 
 * coxa[0],femur[0],tibia[0] = Kaki Kanan Depan
 * coxa[1],femur[1],tibia[1] = Kaki Kanan Belakang
 * coxa[2],femur[2],tibia[2] = Kaki Kiri Depan
 * coxa[3],femur[3],tibia[3] = Kaki Kiri Belakang
 */
 
#define frontRight 0  //Depan Kanan
#define backRight  1  //Belakang Kanan
#define frontLeft  2  //Depan Kiri
#define backLeft   3  //Belakang Kiri

Servo coxa[4], femur[4], tibia[4]; 
float X[4], Y[4], Z[4];
float posX[4], posY[4], posZ[4];
int posCoxa[4], posFemur[4], posTibia[4];
float startX[4], startY[4], endX[4], endY[4], startZ, endZ;

#define button  52
uint8_t led[6] = {A8, A0, A1, A3, A5, A7};

long int timeOut, timeStart;
int yaw, mark[5], value, setpoint, numb = 0, n=0;
int track = 0;
String int_inString = "";
bool start = false;
bool stringComplete = false;  // whether the string is complete

void setup() 
{
  Serial.begin(9600);
  Serial3.begin(9600);

  pinMode(button, INPUT_PULLUP);
  for(int i=0; i<5; i++){pinMode(led[i], OUTPUT);}
  
  for(int leg=0; leg<4; leg++)
  {
    posX[leg] = X[leg] = 0;
    posY[leg] = Y[leg] = 12;
    posZ[leg] = Z[leg] = 5;
  }
  
  coxa[0].attach(17);femur[0].attach(19);tibia[0].attach(21);
  coxa[1].attach(23);femur[1].attach(25);tibia[1].attach(27);
  coxa[2].attach(29);femur[2].attach(31);tibia[2].attach(30);
  coxa[3].attach(28);femur[3].attach(26);tibia[3].attach(24);

  // reserve 200 bytes for the inputString:
//  attachInterrupt(digitalPinToInterrupt(2), serialEvent, CHANGE);
  int_inString.reserve(200);

//  standby
  inverseKinematics(0,0,11,6);
  inverseKinematics(1,0,11,6);
  inverseKinematics(2,0,11,6);
  inverseKinematics(3,0,11,6);  
}

void received_Data() {
  while (Serial.available()) 
  {
    char inChar = (char)Serial.read();
    if(isDigit(inChar)){int_inString+=(char)inChar;} 
    if(inChar == ',')
    {
      value = int_inString.toInt();
      int_inString= "";
      
      mark[0]=value/10000;
      mark[1]=value%10000/1000;
      mark[2]=value%10000%1000/100;
      mark[3]=value%10000%1000%100/10;
      mark[4]=value%10000%1000%100%10%10;     
    }
    if(inChar == '\n')
    {
      yaw = int_inString.toInt(); 
      int_inString= ""; 
      stringComplete = true;   
    }
  }
}

void loop() 
{     
  bool readbutton = digitalRead(button);
  if(!readbutton){delay(300); numb++; timeStart = millis();}
  if(numb<6){digitalWrite(led[numb], HIGH);digitalWrite(led[numb-1], LOW);}
  else{numb=0;for(int i=0; i<6; i++){digitalWrite(led[i], LOW);}}
  
  timeOut = millis()-timeStart;
  if((timeOut > 1200) && (numb!=0)){track = numb; start = true; }

  while(start)
  {
//    received_Data();
//    if (stringComplete) 
//    {
//      Serial3.print(String(yaw) + " ");
//      Serial3.println(String(mark[0]) + String(mark[1]) +  String(mark[2]) + String(mark[3]) + String(mark[4]));
//      stringComplete = false;
//   
    for(int i=0; i<6; i++){digitalWrite(led[i], LOW);}   
    pathFollowing_Strategy();
  }
}
