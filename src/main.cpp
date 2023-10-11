
/*
ref for phase control : https://playground.arduino.cc/Main/ACPhaseControl/
set point is the float voltage for system and must be made 
- max vaLue for pwm is 260 and this value is tested 





*/
#include <Arduino.h>
#include "SevSeg.h"
#include <EEPROM.h>

//--------------------------------------Special Defines---------------------------------------------
SevSeg sevseg; //Instantiate a seven segment object
#define AC_Available_Grid 7
#define AC_Available_Inverter 2
#define Enter A5
#define Up 1
#define Down 0
#define PWM 9	
#define PULSE 4  //trigger pulse width (counts)
#define LED 10


//----------------------------------------Variables-------------------------------------------------
byte A=A1,B=12,C=5,D=3,E=8,F=A0,G=6,H=4;   // define pins 
byte Display_1=A2,Display_2=11,Display_3=13; // define display ports control 
float Battery_Voltage=0,Vin_Battery=0;
unsigned int  ADC_Value=0;  
char txt[32];
//Define Variables we'll be connecting to
double Setpoint, Input, Output; // set point is the desired value for heating 

//Specify the links and initial tuning parameters
double Kp=10, Ki=5,Kd=0;
double highestPowerInverter=50;
uint16_t ScreenTimer=0;
float cutVoltage=12.5;
double PID_Value,PID_P,PID_I,PID_Error;
double HeatingPower=0;
int x=0,y=0;
char SetupProgramNumber=0;
char insideSetup=0;
char SolarMaxPower=0; 
char GridMaxPower=0; 
char PID_MaxHeatingValue;


//--------------------------------------Functions Declartion---------------------------------------
void Read_Battery();
void AC_Control();
void SetFloatVoltage() ; 
void SetCutVoltage();
void SetupProgram();
void EEPROM_Load();
void SetSolarMaxPower();
void CheckForParams();


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
pinMode(LED,OUTPUT); 
attachInterrupt(digitalPinToInterrupt(2),AC_Control,FALLING);

}

//-----------------------------------------Interrupt-------------------------------------------
void AC_Control()
{
 
 /*delayMicroseconds(x); // read AD0
 digitalWrite(PWM, HIGH);
 delayMicroseconds(50);  //delay 50 uSec on output pulse to turn on triac
 digitalWrite(PWM, LOW);
*/
//y=analogRead(A3);
//x=map(y,0,1023,5000,0); // 0 - 10ms 
if (Vin_Battery>=cutVoltage)
{
TCCR1B=0x04; //start timer with divide by 256 input
TCNT1=0;
}
else 
{
 TCCR1B=0x00 ; // stop the timer for no having any output 
 PID_Value=0; 
}

}
ISR(TIMER1_COMPA_vect)
{ //comparator match
   digitalWrite(PWM,HIGH);  //set TRIAC gate to high
   TCNT1 = 65536-PULSE;      //trigger pulse width
}

ISR(TIMER1_OVF_vect){ //timer1 overflow
  digitalWrite(PWM,LOW); //turn off TRIAC gate
  TCCR1B = 0x00;          //disable timer stopd unintended triggers
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
  sevseg.setBrightness(0);  // for clearing flockering in display 
  
  
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

Vin_Battery=sum/10.0;
}
//-------------------------------------Timer for updating screen reads--------------------------
void Segment_Timer_Update ()
{

 TCCR2=0; 
 TCCR2|= (1<<WGM21);   //choosing compare output mode for timer 2
 TCCR2|= (1<<CS22) | (1 <<CS21 ) ;    //choosing 1024 prescalar so we can get 1 ms delay for updating Dipslay
 TIMSK |= (1<<OCIE2);     //enabling interrupt
 OCR2=40;
 }
 ISR(TIMER2_COMP_vect) 
 {
  
    
    TCNT2=0;    // very important 
    ScreenTimer++;
   
    if (ScreenTimer> 0 && ScreenTimer < 9000 && insideSetup==0)
    {
    sevseg.setNumberF(Vin_Battery,1); // Displays '3.141'
    sevseg.refreshDisplay();
    
    }
    if (ScreenTimer>9000 && ScreenTimer< 11000 && insideSetup==0) 
    {
    sevseg.setNumber(HeatingPower); // Displays '3.141' 
    sevseg.refreshDisplay(); 
    }
   
    
    if (ScreenTimer > 11000) ScreenTimer=0; 

 }
//---------------------------------------------------------------------------------
void PID_Compute()
{
  //-> for solar heating power 
if(Vin_Battery>=cutVoltage)
{
 // calculate error 
PID_Error=Vin_Battery-Setpoint; 
 //calculate the p value 
PID_P=Kp*PID_Error; 
if (PID_P <0) PID_P=0;
if (PID_P > PID_MaxHeatingValue) PID_P=PID_MaxHeatingValue; 
// calculate the I controller 
PID_I=PID_I+ (Ki*PID_Error);
if (PID_I <0) PID_I=0;
if (PID_I > PID_MaxHeatingValue) PID_I=PID_MaxHeatingValue; 
// calcaulte the pid value final 
PID_Value=PID_P+PID_I ; 
// to make range of pid 
if (PID_Value <0) PID_Value=0;
if (PID_Value > PID_MaxHeatingValue) PID_Value=PID_MaxHeatingValue; 

HeatingPower=map(PID_Value,0,260,0,100); // map pid value show the range between 1- 260 what is the power 
OCR1A=map(PID_Value,0,PID_MaxHeatingValue,PID_MaxHeatingValue,1); // minus value of pwm is 1 and max value is 260 
}
else 
{
  PID_Value=0; 
  // we also can stop timer to make output zero but i have done it in interrupts 
}
}
//-------------------------------------------Timer Init---------------------------------------
void Timer_Init()
{
TCCR1A=0 ;  // zero timer 
TCCR1B=0;  // zero timer 
TCCR1B |= (1<<WGM12); 
TIMSK  |= (1 <<OCIE1A) | (1<< TOIE1) ;   // TIMER OVERFLOW AND INTERRUPT ENABLE 
}
//--------------------------------------CheckForSetup------------------------------------------
void CheckForSet()
{

  
  if (digitalRead(Enter)==1)
  {
    insideSetup=1;
    delay(100);
    SetupProgram() ; 
  }


}
//----------------------------------------Setup Program------------------------------
void SetupProgram()
{
while (digitalRead(Enter)==1)
{
    SetCutVoltage(); 
    delay(500); 
    SetFloatVoltage(); 
    delay(500);
    SetSolarMaxPower();
    delay(500);
    break;
}
insideSetup=0;

}

//----------------------------------------Set Cut Voltage--------------------------------------
void SetCutVoltage()
{
delay(500);
while(digitalRead(Enter)==0)
{
 sevseg.setChars("P01"); 
 sevseg.refreshDisplay(); 
} 
delay(500);
while (digitalRead(Enter)==0)
{
sevseg.setNumberF(cutVoltage,1); 
sevseg.refreshDisplay(); 
while (digitalRead(Up)==1 || digitalRead(Down)==1) 
{

 sevseg.setNumberF(cutVoltage,1); 
 sevseg.refreshDisplay();

 if (digitalRead(Up)==1) 
 {
  delay(100);
  cutVoltage+=0.1;

 }
  if (digitalRead(Down)==1) 
 {
  delay(100);
  cutVoltage-=0.1;
   }
} // end while up and down
}  // end main while 
EEPROM.put(0,cutVoltage);
}
//---------------------------------------Set Float Vooltage----------------------------------
void SetFloatVoltage()
{
delay(500);
while(digitalRead(Enter)==0)
{
 sevseg.setChars("P02"); 
 sevseg.refreshDisplay(); 
} 
delay(500);
while (digitalRead(Enter)==0)
{
sevseg.setNumberF(Setpoint,1); 
sevseg.refreshDisplay(); 
while (digitalRead(Up)==1 || digitalRead(Down)==1) 
{

 sevseg.setNumberF(Setpoint,1); 
 sevseg.refreshDisplay();

 if (digitalRead(Up)==1) 
 {
  delay(100);
  Setpoint+=0.1;

 }
  if (digitalRead(Down)==1) 
 {
  delay(100);
  Setpoint-=0.1;
   }
} // end while up and down
}  // end main while 
EEPROM.put(4,Setpoint);
}
//------------------------------------------Set Solar Max Power----------------------------------
void SetSolarMaxPower()
{
 delay(500);
while(digitalRead(Enter)==0)
{
 sevseg.setChars("P03"); 
 sevseg.refreshDisplay(); 
} 
delay(500);
while (digitalRead(Enter)==0)
{
sevseg.setNumber(SolarMaxPower); 
sevseg.refreshDisplay(); 
while (digitalRead(Up)==1 || digitalRead(Down)==1) 
{

 sevseg.setNumberF(SolarMaxPower,1); 
 sevseg.refreshDisplay();

 if (digitalRead(Up)==1) 
 {
  delay(100);
  SolarMaxPower++;

 }
  if (digitalRead(Down)==1) 
 {
  delay(100);
  SolarMaxPower++;
   }
  if (SolarMaxPower>100)  SolarMaxPower=0;
  if (SolarMaxPower<0) SolarMaxPower=0;
} // end while up and down
}  // end main while 
PID_MaxHeatingValue=map(SolarMaxPower,0,100,0,260);
EEPROM.write(8,SolarMaxPower); 
EEPROM.write(9,PID_MaxHeatingValue); // for solar this value 

}
//-----------------------------------------EEPROM Load------------------------------------------
void EEPROM_Load()
{
EEPROM.get(0,cutVoltage);
EEPROM.get(4,Setpoint); 
SolarMaxPower=EEPROM.read(8); 
PID_MaxHeatingValue=EEPROM.read(9);


}
//---------------------------------------CheckForParams-----------------------------------------
void CheckForParams()
{
if (cutVoltage<0 || cutVoltage>70 || isnan(cutVoltage) ) 
{
  cutVoltage=0;
  EEPROM.put(0,cutVoltage); 
  EEPROM_Load();
}

if (Setpoint<0 || Setpoint>70 || isnan(Setpoint) ) 
{
  Setpoint=0;
  EEPROM.put(4,Setpoint); 
  EEPROM_Load();
}

if (SolarMaxPower<0 || Setpoint>100) 
{
  SolarMaxPower=0;
  EEPROM.put(8,SolarMaxPower); 
  EEPROM_Load();
}
if (PID_MaxHeatingValue<0 || PID_MaxHeatingValue>260) 
{
  PID_MaxHeatingValue=0;
  EEPROM.put(9,PID_MaxHeatingValue); 
  EEPROM_Load();
}

}
//*****************************************MAIN LOOP********************************************
void setup() {
  // put your setup code here, to run once:
Segment_Init();
Segment_Timer_Update();
GPIO_Init();  
EEPROM_Load();
Timer_Init(); 

}
//-> start developing
void loop() {
  // put your main code here, to run repeatedly:
   CheckForParams();
   CheckForSet();
   Read_Battery();
   PID_Compute();
   delay(200);


 

}
