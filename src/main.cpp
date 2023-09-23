#include <Arduino.h>
#include "SevSeg.h"

//--------------------------------------Special Defines---------------------------------------------
SevSeg sevseg; //Instantiate a seven segment object



//----------------------------------------Variables-------------------------------------------------
float value;
float voltage;
int voltage_int;
byte A=A1,B=12,C=5,D=3,E=8,F=A0,G=6,H=4;   // define pins 
byte Display_1=A2,Display_2=11,Display_3=13; // define display ports control 
//--------------------------------------Functions Declartion---------------------------------------
void GPIO_Init()
{

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

//*****************************************MAIN LOOP********************************************
void setup() {
  // put your setup code here, to run once:
Segment_Init();

}
//-> start developing
void loop() {
  // put your main code here, to run repeatedly:
  sevseg.setNumber(310); // Displays '3.141'
  sevseg.refreshDisplay();

    
}
