/////////Ball and Plate///////////////////////////////
/*
BALL AND PLATE PID CONTROL
*/
//////////////////////////////////////////////////////
///Libraries///


//point 485 190


#include <stdlib.h>
#include <PID_v1.h>
#include <stdint.h>
//#include "TouchScreen.h"
#include <SPI.h>
#include <Wire.h>
//#include <wiinunchuk.h>
#include<Servo.h>
// Definitions TOUCH PINS

#define MacSerial Serial 
#define PiSerial Serial2

#define YP A0 //0
#define XM A1 //1
#define YM 3  //3
#define XP 4 //4

int flagflag=0;
int temp=6;

//TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);
int buttonPushCounter = 1;   // counter for the number of button presses
int lastButtonState = 0;     // previous state of the button
int flag , flagZ ;


float xVal , yVal ;
int cCount=0;
int flagC=0;
int flagK=0;
float kk=0;
int fl=0;
double l =0.00;
unsigned int noTouchCount = 0; //viariable for noTouch
double  k=0;
// PID values
double Setpoint, Input, Output; //for X
double Setpoint1, Input1, Output1; //for Y
//
int Modulo;
long lastcas=0;
// servos variables
Servo servo1; //X axis
Servo servo2; //Y axis

String serialDate = "";
String InputDate = "";
String Input1Date = "";


uint16_t homeX = 550;            // raw data value for center of touchscreen
uint16_t homeY = 550;            // raw data value for center of touchscreen             

//uint16_t homeX = 65;
//uint16_t homeY = 65;

//float convertX = 151.0 / 955.0;  // converts raw x values to mm. found through manual calibration
//float convertY = 91.0 / 927.0;   // converts raw y values to mm. found through manual calibration

float convertX = 650.0 / 950.0;
float convertY = 650.0 / 950.0;


/////TIME SAMPLE
//int Ts = 50; 
int Ts = 50;
unsigned long Stable=0; 
//PID const
float Kp = 0.3;                                                     
float Ki = 0.03;                                                      
float Kd = 0.13;

float Kp1 = 0.3;                                                       
float Ki1 = 0.08;                                                      
float Kd1 = 0.13;
long cas=0; 
//INIT PID
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
PID myPID1(&Input1, &Output1, &Setpoint1,Kp1,Ki1,Kd1, DIRECT);

struct p{int x = 20; int y = 30; int z = 400;};

void setup()
{
  servo1.attach(5);
  servo2.attach(6);
  Output=50;
  Output1=80;
  servo1.write(Output);
  servo2.write(Output1);
  
  //init NUN
  //nunchuk_setpowerpins();
  //nunchuk_init();
  //nunchuk_get_data(); 
 
  //INIT PINS
  pinMode(9, OUTPUT);
  pinMode(8, OUTPUT);
  digitalWrite(9,LOW); //LED INIT
  digitalWrite(8,LOW);

  PiSerial.begin(9600);
  MacSerial.begin(9600);
  
  //INIT OF TOUSCHSCREEN
   //TSPoint p = ts.getPoint();
   
 //Input=120;
 //Input1=65;
  
  Input=550;
  Input1=550;
  
  //INIT SETPOINT
  //Setpoint=120;
  //Setpoint1=65;
    Setpoint=65;
    Setpoint1=65;
  
  
  //// Make plate flat
  servo1.attach(5); 
  servo2.attach(6);
  //Output=95;
  //Output1=95;
  Output=90;
  Output1=90;
  
  servo1.write(Output);
  servo2.write(Output1);
  
  //Zapnutie PID
  myPID.SetMode(AUTOMATIC);
  //myPID.SetOutputLimits(20, 160);
   myPID.SetOutputLimits(40, 100);
  myPID1.SetMode(AUTOMATIC);
  //myPID1.SetOutputLimits(20, 160);
    myPID1.SetOutputLimits(40, 100);
  // TIME SAMPLE
  myPID1.SetSampleTime(Ts); 
  myPID.SetSampleTime(Ts);  
  /////
  delay(100);
  
 
  ///
 }
 
void loop()
{
  //Stable = 125!!!!!
  while(Stable<20) //REGULATION LOOP
  {
    MacSerial.print("\nin the Not Stable Loop!\n");
   //TSPoint p = ts.getPoint();   //measure pressure on plate
   //struct p{int x = 20; int y = 30; int z = 400;};
   int z = 400;
   //if (z > ts.pressureThreshhold) //ball is on plate
   if (z > 300) //ball is on plate
   {  
      servo1.attach(5); //connect servos
      servo2.attach(6); 
      //setDesiredPosition();  
         Setpoint=500;
         Setpoint1=500;
      
      
      noTouchCount = 0;  
      //TSPoint p = ts.getPoint(); // measure actual position 
      //stuct p{int x = 20; int y = 30; int z = 400;};
//      Input=(p.x * convertX);  // read and convert X coordinate
//      Input1=(p.y * convertY); // read and convert Y coordinate
     //Input=(20 * convertX);
      //Input1=(30 * convertY);



//get the position from Raspberry Pi by the Camera
      //serialDate = "";
      //InputDate = "";
      //Input1Date = "";

      PiSerial.flush();

    MacSerial.print(PiSerial.available());

   while(PiSerial.available() == 0)
      {
        MacSerial.print("****1****");

      }

      
      while (PiSerial.available() > 0)
      {
            if (PiSerial.read() == 's')
            { 
  //            MacSerial.print("111111111");
              serialDate = "";
             while(temp > 0) {
              MacSerial.print("*");
              
              serialDate += char(PiSerial.read());
              delay(2);
              temp -= 1;
              flagflag = 1;
             }
             
             temp = 6;

            if(flagflag) {flagflag = 0; break;}
//              else {continue;}
            }
 
      }

      if( serialDate != "" ) {
      InputDate = "";
      InputDate += serialDate[0];
      InputDate += serialDate[1];
      InputDate += serialDate[2];
      Input = InputDate.toInt();
      
      Input1Date = "";
      Input1Date += serialDate[3];
      Input1Date += serialDate[4];
      Input1Date += serialDate[5];
      Input1 = Input1Date.toInt();
      }
      

      MacSerial.print("\n\nThe x: ");
      MacSerial.print(Input);
      MacSerial.print("\nThe y: ");
      MacSerial.print(Input1);

      Input=(Input * convertX);
      Input1=(Input1 * convertY);

      MacSerial.print("\nThe x converted: ");
      MacSerial.print(Input);
      MacSerial.print("\nThe y converted: ");
      MacSerial.print(Input1);

          if((Input>Setpoint-2 && Input<Setpoint+2 && Input1>Setpoint1-2 && Input1<Setpoint1+2))//if ball is close to setpoint
          {
              Stable=Stable+1; //increment STABLE
              digitalWrite(9,HIGH);
                 
          }
          else
          {
              digitalWrite(9,LOW);
          }
       myPID.Compute();  //action control X compute
       myPID1.Compute(); //   action control  Y compute   
    
  }
   else //if there is no ball on plate
  {
    noTouchCount++; //increment no touch count

    if(noTouchCount == 75) 
    {
     noTouchCount++; 
     //Output=95; //make plate flat
     //Output1=93;

     Output=70;
     Output1=70;
     
     servo1.write(Output); 
     servo2.write(Output1);
    }
    if(noTouchCount == 150) //if there is no ball on plate longer
    {
     servo1.detach(); //detach servos
     servo2.detach();     
   
    }
  } //else end
  servo1.write(Output);//control
  servo2.write(Output1);//control 
  //MacSerial.print(Setpoint);   Serial.print(",");  Serial.print(Setpoint1);  Serial.print(",");  Serial.print(Input);Serial.print(","); Serial.println(Input1); 
     
  }////END OF REGULATION LOOP///
  
  servo1.detach();//detach servos
  servo2.detach();
  
 ///KONTROLA STABILITY////
 while(Stable==20)//if is stable
 {
   MacSerial.print("\nin the Stable Loop!\n");
  //still measure actual postiion
    //setDesiredPosition(); 
    Setpoint=105;
    Setpoint1=105;
    
    //TSPoint p = ts.getPoint();
//stuct p{int x = 20; int y = 30; int z = 400;};
    
//    Input=(p.x * convertX);  // read and convert X coordinate
//Input=(20 * convertX);
  //    Input1=(30 * convertY);

//get the position from Raspberry Pi by the Camera
      
      //serialDate = "";
      //InputDate = "";
      //Input1Date = "";

      PiSerial.flush();

      while(PiSerial.available() == 0)
      {
        //MacSerial.print("&&&");
        continue;  
      }

     
      while (PiSerial.available() > 0)
      {
            if (PiSerial.read() == 's')
            { 
  //            MacSerial.print("111111111");
              serialDate = "";
             while(temp > 0) {
              MacSerial.print("*");
              
              serialDate += char(PiSerial.read());
              delay(2);
              temp -= 1;
              flagflag = 1;
             }
             
             temp = 6;

            if(flagflag) {flagflag = 0; break;}
//              else {continue;}
            }
 
      }

      if( serialDate != "" ) {
      InputDate = "";
      InputDate += serialDate[0];
      InputDate += serialDate[1];
      InputDate += serialDate[2];
      Input = InputDate.toInt();
      
      Input1Date = "";
      Input1Date += serialDate[3];
      Input1Date += serialDate[4];
      Input1Date += serialDate[5];
      Input1 = Input1Date.toInt();
      }
      

      MacSerial.print("\n\nThe x: ");
      MacSerial.print(Input);
      MacSerial.print("\nThe y: ");
      MacSerial.print(Input1);

      Input=(Input * convertX);
      Input1=(Input1 * convertY);

      MacSerial.print("\nThe x converted: ");
      MacSerial.print(Input);
      MacSerial.print("\nThe y converted: ");
      MacSerial.print(Input1);
    
    if(Input<Setpoint-2 || Input>Setpoint+2 || Input1>Setpoint1+2 || Input1<Setpoint1-2  ) //if ball isnt close to setpoint
    {
      servo1.attach(5); //again attach servos
      servo2.attach(6);
      digitalWrite(9,LOW);
      Stable=0; //change STABLE state
    }
    
  }//end of STABLE LOOP
}//loop end

////////////////////////Functions////////////////// 
///// DESIRED POSITION
void setDesiredPosition()
{
 
  
 //nunchuk_get_data(); 
 //if zbutton is pressed, zero positions
 
 //int c = nunchuk_zbutton();
 int c = 3;
 if (c != lastButtonState) {
    // if the state has changed, increment the counter
  if (c == HIGH && digitalRead(11)==0 ) {
      // if the current state is HIGH then the button
      // wend from off to on:
      buttonPushCounter++;   
    }
  }
   lastButtonState =c;
   
   switch (buttonPushCounter)
   {
    case 1:
    Setpoint=120;
    Setpoint1=70;
    fl=1;
    break;
    case 2:
    Setpoint=52;
    Setpoint1=70;
    fl=2;
    break;
    case 3:
    Setpoint=52;
    Setpoint1=40;
    fl=3;
    break; 
    case 4:
    Setpoint=120;
    Setpoint1=40;
    buttonPushCounter=0;
    fl=4;
    break;  
   }  
    //if (nunchuk_cbutton()&&fl==1)///LEMNISCATE TRAJECOTRY
  if (0)
  {
    Setpoint = 85+ (50*cos(k))/(1+sin(k)*sin(k));
    Setpoint1 = 55+ (50*sin(k)*cos(k))/(1+sin(k)*sin(k));
    buttonPushCounter=0;
    k=k+0.008; 
  }
  //if (nunchuk_cbutton()&&fl==2)// CIRCLE TRAJECTORY
  if (0)
  {
    Setpoint = 85+ 25*cos(k);
    Setpoint1 = 55+ 25*sin(k);
    buttonPushCounter=0;
    k=k-0.02; 
  }
  //if (nunchuk_cbutton()&&fl==3)/// ELLIPSE TRAJECORY
  if (0)
  {
    Setpoint = 85+ 40*cos(k);
    Setpoint1 = 55+ 25*sin(k);
    buttonPushCounter=0;
    k=k-0.02; 
  }
  //if (nunchuk_cbutton()&&fl==4) //PENTAGRAM TRAJECOTRY
  if (0)
  {
    Setpoint =85+  18*cos(k)+12*cos(k*150);//
    Setpoint1 =55+ 18*sin(k)-12*sin(k*150);//
    buttonPushCounter=0;
    k=k+0.01; 
  }
}
 
