#include <Arduino.h>
#include "SevSeg.h"

//--------------------------------------Special Defines---------------------------------------------
SevSeg sevseg; //Instantiate a seven segment object
#define AC_Available_Grid 2
#define AC_Available_Inverter 7
#define Enter A5
#define Up 1
#define Down 0
//----------------------------------------Variables-------------------------------------------------
byte A=A1,B=12,C=5,D=3,E=8,F=A0,G=6,H=4;   // define pins 
byte Display_1=A2,Display_2=11,Display_3=13; // define display ports control 
float Battery_Voltage=0,Vin_Battery=0;
unsigned int  ADC_Value=0;  
char txt[32];
//--------------------------------------Functions Declartion---------------------------------------
void Read_Battery();




//-------------------------------------------Functions---------------------------------------------- 
void GPIO_Init()
{
pinMode(AC_Available_Grid,INPUT);
pinMode(AC_Available_Inverter,INPUT);
pinMode(Enter,INPUT);
pinMode(Up,INPUT);
pinMode(Down,INPUT);
pinMode(A3,INPUT);  // battery voltage reading 
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
 noInterrupts();
 TCCR2=0; 
 TCCR2|= (1<<WGM21);   //choosing compare output mode for timer 2
 TCCR2|=(1<<CS22) | (1 <<CS21 ) | ( 1<< CS20) ;    //choosing 1024 prescalar so we can get 1 ms delay for updating Dipslay
 OCR2=40;
 TIMSK |= (1<<OCIE2);     //enabling interrupt
 TIMSK |= (1<<OCF2); 
 interrupts(); 
}
 ISR(TIMER2_COMP_vect) 
 {
    TCNT2=0;    // very important 
    sevseg.setNumberF(Vin_Battery,1); // Displays '3.141'
    sevseg.refreshDisplay();


 }
//*****************************************MAIN LOOP********************************************
void setup() {
  // put your setup code here, to run once:
Segment_Init();
GPIO_Init(); 
Segment_Timer_Update();

}
//-> start developing
void loop() {
  // put your main code here, to run repeatedly:
 Read_Battery();
 //sevseg.setNumber(314); // Displays '3.141'
 //sevseg.refreshDisplay();   
}
