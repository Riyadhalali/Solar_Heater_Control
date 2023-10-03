#include <Arduino.h>
#include "SevSeg.h"

//--------------------------------------Special Defines---------------------------------------------
SevSeg sevseg; //Instantiate a seven segment object
#define AC_Available_Grid 7
#define AC_Available_Inverter 2
#define Enter A5
#define Up 1
#define Down 0
#define PWM 9	

//----------------------------------------Variables-------------------------------------------------
byte A=A1,B=12,C=5,D=3,E=8,F=A0,G=6,H=4;   // define pins 
byte Display_1=A2,Display_2=11,Display_3=13; // define display ports control 
float Battery_Voltage=0,Vin_Battery=0;
unsigned int  ADC_Value=0;  
char txt[32];
//Define Variables we'll be connecting to
double Setpoint=27.5, Input, Output; // set point is the desired value for heating 
//Specify the links and initial tuning parameters
double Kp=10, Ki=5,Kd=0;
double highestPowerInverter=50;
uint16_t ScreenTimer=0;
float cutVoltage=25.0;
double PID_Value,PID_P,PID_I,PID_Error;
double HeatingPower=0;
int x=0,y=0;
//--------------------------------------Functions Declartion---------------------------------------
void Read_Battery();
void AC_Control();



//-------------------------------------------Functions---------------------------------------------- 
void GPIO_Init()
{
pinMode(AC_Available_Grid,INPUT);
pinMode(AC_Available_Inverter,INPUT);
pinMode(Enter,INPUT);
pinMode(Up,INPUT);
pinMode(Down,INPUT);
pinMode(PWM,OUTPUT);
pinMode(A3,INPUT);  // battery voltage reading 
attachInterrupt(digitalPinToInterrupt(2),AC_Control,FALLING);
Serial.begin(9600);
}

//-----------------------------------------Interrupt-------------------------------------------
void AC_Control()
{
  delayMicroseconds(x); // read AD0
  digitalWrite(PWM, HIGH);
  delayMicroseconds(50);  //delay 50 uSec on output pulse to turn on triac
  digitalWrite(PWM, LOW);
}
//----------------------------------------7 Segment Init----------------------------------------
void Segment_Init()
{
  byte numDigits = 3;
  byte digitPins[] = {A2, 11, 13};
  byte segmentPins[] = {A, B, C, D, E, F, G, H};
  bool resistorsOnSegments = false; // 'false' means resistors are on digit pins
  byte hardwareConfig = COMMON_ANODE; // See README.md for options
  bool updateWithDelays = false; // Default 'false' is Recommended
  bool leadingZeros = false; // Use 'true' if you'd like to keep the leading zeros
  bool disableDecPoint = false; // Use 'true' if your decimal point doesn't exist or isn't connected. Then, you only need to specify 7 segmentPins[]
  sevseg.begin(hardwareConfig, numDigits, digitPins, segmentPins, resistorsOnSegments,
  updateWithDelays, leadingZeros, disableDecPoint);
}
//---------------------------------------Read Battery Voltage-----------------------------------
void Read_Battery()
{
float sum=0 , Battery[10];
float ADC_PID=512;
ADC_Value=analogRead(A3);

Battery_Voltage=(ADC_Value *5.0)/1024.0;

for ( char i=0; i<10 ; i++)
{
Battery[i]=((10.5/0.5)*Battery_Voltage);
delay(50);
sum+=Battery[i];
}

Vin_Battery= sum/10.0;

}
//-------------------------------------Timer for updating screen reads--------------------------
void Segment_Timer_Update ()
{
  

 TCCR2=0; 
 TCCR2|= (1<<WGM21);   //choosing compare output mode for timer 2
 TCCR2|=(1<<CS22) | (1 <<CS21 ) | ( 1<< CS20) ;    //choosing 1024 prescalar so we can get 1 ms delay for updating Dipslay
 OCR2=10;
 TIMSK |= (1<<OCIE2);     //enabling interrupt
 TIMSK |= (1<<OCF2); 

}
 ISR(TIMER2_COMP_vect) 
 {

    TCNT2=0;    // very important 
    ScreenTimer++;
  
   if (ScreenTimer> 0 && ScreenTimer < 1000)
    {
    sevseg.setNumberF(Vin_Battery,1); // Displays '3.141'
    sevseg.refreshDisplay();
    }
    if (ScreenTimer>1000 && ScreenTimer< 2000) 
    {
    sevseg.setNumber(HeatingPower); // Displays '3.141'
    sevseg.refreshDisplay(); 
    }
    if (ScreenTimer > 2000) ScreenTimer=0; 
    
  
 }
  //-----------------------------Write PWM----------------------------------------
void PWM_Init()
{
TCCR1A=0;
TCCR1B=0;
TCCR1A |= (1<<COM1A1)  ; //clear OC1A/OC1B on Compare Match and set mode 14 
TCCR1A |= (1 << WGM11);
TCCR1B |= (1 << WGM12);
TCCR1B |= (1 << WGM13);
TCCR1B |= (1<<CS10) | (1<<CS11) ; // prescalar 64 
ICR1=2500;
}


void PID_Compute()
{
 // calculate error 
PID_Error=Vin_Battery-Setpoint; 
 //calculate the p value 
PID_P=Kp*PID_Error; 
if (PID_P <0) PID_P=0;
if (PID_P > 5000) PID_P=5000; 
// calculate the I controller 
PID_I=PID_I+ (Ki*PID_Error);
if (PID_I <0) PID_I=0;
if (PID_I > 5000) PID_I=5000; 
// calcaulte the pid value final 
PID_Value=PID_P+PID_I ; 
// to make range of pid 
if (PID_Value <0) PID_Value=0;
if (PID_Value > 5000) PID_Value=5000; 
//analogWrite(PWM,PID_Value) ; 
//OCR1A=PID_Value;
HeatingPower=map(PID_Value,0,5000,0,100); // map pid value 
x=map(PID_Value,0,5000,5000,0);

}
//*****************************************MAIN LOOP********************************************
void setup() {
  // put your setup code here, to run once:
Segment_Init();
GPIO_Init(); 
Segment_Timer_Update();
//PWM_Init(); 
}
//-> start developing
void loop() {
  // put your main code here, to run repeatedly:
   Read_Battery();
   PID_Compute();
   //y=analogRead(A3);
   //x=map(y,0,1023,5000,0); // 0 - 10ms 

}
