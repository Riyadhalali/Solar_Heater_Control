
/*
ref for phase control : https://playground.arduino.cc/Main/ACPhaseControl/
set point is the float voltage for system and must be made 
- max vaLue for pwm is 260 and this value is tested 

*/
#include <Arduino.h>
#include "SevSeg.h"
#include <EEPROM.h>
#include <OneButton.h>

//--------------------------------------Special Defines---------------------------------------------
SevSeg sevseg; //Instantiate a seven segment object

#define AC_Available_Grid 3
#define AC_Available_Inverter 2
#define Enter A5
#define Up 1
#define Down 0
#define PWM 9	
#define PULSE 4//trigger pulse width (counts)
#define OCR1A_MaxValue 245
#define PIDMaxValue 2000 // pid  value just for selecting the max range 
#define Fan A4
#define Contactor 8

OneButton button = OneButton(
  Enter,  // Input pin for the button
  false,       // Button is active high
  false        // Disable internal pull-up resistor
);
//----------------------------------------Variables-------------------------------------------------
byte A=A1,B=12,C=5,D=7,E=10,F=A0,G=6,H=4;   // define pins 
byte Display_1=A2,Display_2=11,Display_3=13; // define display ports control 
double Battery_Voltage=0,Vin_Battery=0;
unsigned int  ADC_Value=0;  
char txt[32];
//Define Variables we'll be connecting to
double Setpoint, Input, Output; // set point is the desired value for heating 
//Specify the links and initial tuning parameters
double Kp=20,Ki=20,Kd=0;
uint16_t ScreenTimer=0;
float cutVoltage=0;
float PID_Value,PID_P,PID_I,PID_Error;
double HeatingPower=0;
int x=0,y=0;
char SetupProgramNumber=0;
char SetupProgramNumberVariables=0;  // this variable to display the variable of the program when entering program 
char insideSetup=0;
unsigned int SolarMaxPower=0,UtilityMaxPower=0;
char GridMaxPower=0; 
float PID_MaxHeatingValue;
float PID_MaxHeatingValueUtility;
unsigned long lastTime,lastTime_2;// for timing in PID controller 
int PWM_Value=OCR1A_MaxValue;   // start value 
unsigned long longPressTime = 500; // Duration of a long press in milliseconds
boolean loopRunning = false; // Flag indicating whether the loop is running
bool InProgramMode=false; 
char LoadsAlreadySwitchOff=0;
unsigned long now,now_2,lastIncreaseTime=0,lastDecreaseTime=0;
unsigned long readBatteryNow; 
double timeChange,timeChange_2,timeChangeBattery;
char CountRealTimeSeconds=0; // to start counting real time seconds 
unsigned int SecondsReadTime=0;
unsigned int overflowTimes=0; 
int ledState=LOW;
unsigned long previousMillis = 0;  // will store last time LED was updated
unsigned long currentMillis=0;
// constants won't change:
const long interval = 1000;  // interval at which to blink (milliseconds)
char fanState=0;
unsigned long currentMillisFan,previousMillisFan,currentMillisBattery,prevoiusMillisBattery;
char secondsFan=0; 
char fanTimeOff=5;  // fan time to turn off is 60 seconds 
char fanTimeOn=30;
char SystemBatteryMode=0; // for checking the battery system  mode 
char displayResetMessage=0;
unsigned int esc=0;  
double VinBatteryError=0.0;
double VinBatteryDifference=0.0;
char addError=0;
double Vin_Battery_Calibrated=0.0;   // this is the reading voltage 
char displayWelcomeScreen=0,displayVersionNumber=0;
char  contactorEnableLowBattery=0;   
char MiniSolarPowerStart=5; 
uint8_t DelayTime=0; 
float accelerationValueUtility=0; 
char timetoReachMax=90 ; // time to make the utilty reach it max value 
char amplifeError=2; // to make error bigger when battery voltage is equal 
char fanisOn=0;
double IncreaseTimeInSeconds = 1;  // Time interval for increasing PWM_Value
double DecreaseTimeInSeconds = 1;  // Time interval for decreasing PWM_Value
float Vin_Battery_=0.0;
//--------------------------------------Functions Declartion---------------------------------------
void Read_Battery();
void AC_Control();
void SetFloatVoltage() ; 
void SetCutVoltage();
void SetupProgram();
void EEPROM_Load();
void SetSolarMaxPower();
void CheckForParams();
void Press_Detect();
void SetIncreaseTime();
void longPress();
void shortPress(); 
void SetUtilityMaxPower();
void PID_ComputeForUtility();
void CheckForGrid();
void factorySettings();
void Timer_Seconds();
void SetDelayTime();
void Grid_Turn_Off();
void checkFan();
void CheckSystemBatteryMode();
void EEPROM_FactorySettings();
void SetCalibrationVoltage();
void WelcomeScreen();
void checkCutOffVoltage();
void checkContactor();
void SetContactorLatch(); // enable turn off contactor when battery voltage is low 
void Screen();  
void SetDecreaseTime();
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
pinMode(Fan,OUTPUT); 
pinMode(Contactor,OUTPUT);
attachInterrupt(digitalPinToInterrupt(2),AC_Control,FALLING);
attachInterrupt(digitalPinToInterrupt(3),Grid_Turn_Off,RISING);
button.setLongPressIntervalMs(100);
button.attachLongPressStart(Press_Detect); // Attach a function to the long press stop event
}
//-----------------------------------------Grid Switch Off-------------------------------------
void Grid_Turn_Off()
{
CheckForGrid();
}
//--------------------------------------------Interrupt-------------------------------------------
void AC_Control()
{

//-> if grid is not available 
if (digitalRead(AC_Available_Grid)==1)
{ 
if (Vin_Battery_Calibrated>cutVoltage )
{
//if   (PWM_Value==0) PWM_Value=1;
//if ( HeatingPower > 5) OCR1A=PWM_Value;

//*******************************************************************
if (PWM_Value<OCR1A_MaxValue && HeatingPower > 50)
{
TCCR1B=0x04; //start timer with divide by 256 input
TCNT1 = 0;   //reset timer - count from zero
} 
}
else  if (Vin_Battery_Calibrated<=cutVoltage)
{
 SecondsReadTime=0;  // make time zero for not starting and stoping 
 TCCR1B=0x00 ; // stop the timer for no having any output 
 digitalWrite(PWM,LOW);
 PID_Value=0; 
 PID_I=0;
 PID_P=0;
}
//*******************************END IF*******************************

//-> Making range of pwm_value and it must not be zero
if (PWM_Value>=OCR1A_MaxValue ) 

{
  ///>@ very important 
 digitalWrite(PWM,LOW); // if this line was not exists then when heating power reaches 0 then the output will become max because there is nothing driving the triac
 TCCR1B=0x00; //start timer with divide by 256 input ; // stop the timer for no having any output 
 PWM_Value=OCR1A_MaxValue;
 OCR1A=PWM_Value; // to make smooth ouput
}
 else if(PWM_Value<OCR1A_MaxValue  && HeatingPower > 50) 
{
//-> check that range of PWM_value is between 1 and 255
if (PWM_Value==0) PWM_Value=1; // can't be zero because it will give sto the output 
OCR1A=PWM_Value;
}
}// end if ac grid 
//------------------------------------------END OF GRID NOT AVAILABLE-------------------------------
else if (digitalRead(AC_Available_Grid)==0)
{

//-> check that range of PWM_value is between 1 and 255
//if (PWM_Value==0) PWM_Value=1; // can't be zero because it will give sto the output 
//OCR1A=PWM_Value;
//***********************NEW PART FOR AC GRID**********************************
if (PWM_Value>=OCR1A_MaxValue ) 
{
///>@ very important 
 digitalWrite(PWM,LOW); // if this line was not exists then when heating power reaches 0 then the output will become max because there is nothing driving the triac
 TCCR1B=0x00; //start timer with divide by 256 input ; // stop the timer for no having any output 
 PWM_Value=OCR1A_MaxValue;
 OCR1A=PWM_Value; // to make smooth ouput
}
 else if(PWM_Value<OCR1A_MaxValue  && HeatingPower > 50) 
{
TCCR1B=0x04; //start timer with divide by 256 input
TCNT1 = 0;   //reset timer - count from zero
//-> check that range of PWM_value is between 1 and 255
if (PWM_Value==0) PWM_Value=1; // can't be zero because it will give sto the output 
OCR1A=PWM_Value;
}
}  // END AC GRID AVAILABLE 
}  // end function 
//---------------------------------END INTERRUPT------------------------------------
ISR(TIMER1_COMPA_vect)
{ 
  //comparator match
  //-
  if (PWM_Value< OCR1A_MaxValue && PWM_Value >1) 
  {
   digitalWrite(PWM,HIGH);     //set TRIAC gate to high
   TCNT1 = 65536-PULSE;       //trigger pulse width
  } 
  else         // WHEN LOAD IS 100% to make full output 
  {
   digitalWrite(PWM,HIGH);    //set TRIAC gate to high
  } 
 
  

 }

ISR(TIMER1_OVF_vect){       //timer1 overflow

 if (PWM_Value <OCR1A_MaxValue && PWM_Value >1 )
 {
  digitalWrite(PWM,LOW);    //turn off TRIAC gate
  TCCR1B = 0x00;            //disable timer stopd unintended triggers
 } 

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

void Read_Battery() {
    const int numSamples = 100;
    float sum = 0;
    float Battery[numSamples];
    float fanError=0; 

    for (int i = 0; i < numSamples; i++) {
        ADC_Value = analogRead(A3);
        Battery_Voltage = (ADC_Value * 5.0) / 1023.0;
        Battery[i] = ((10.5 / 0.5) * Battery_Voltage);
        sum += Battery[i];
        delay(10);
    }

    Vin_Battery = sum / numSamples;

    if (fanisOn==1) fanError=0.065 ; else (fanError=0);  // for fan error 

    if (addError == 1) {
        Vin_Battery_Calibrated = Vin_Battery + VinBatteryDifference + fanError;  // must add error fan 
    } else {  // addError = 0
        Vin_Battery_Calibrated = Vin_Battery - VinBatteryDifference + fanError;  // must add error fan 
    }
}
//-------------------------------------Timer for updating screen reads--------------------------
void Segment_Timer_Update ()
{
 TCCR2B=0; 
 TCCR2B|= (1<<WGM21);   //choosing compare output mode for timer 2
 TCCR2B|= (1<<CS22) | (1 <<CS21 ) ;    //choosing 1024 prescalar so we can get 1 ms delay for updating Dipslay
 TIMSK2 |= (1<<OCIE2A);     //enabling interrupt
 OCR2A=20;
 }
 
 ISR(TIMER2_COMPA_vect) 
 {

    TCNT2=0;    // very important 
    ScreenTimer++;
    if (SetupProgramNumber==1) 
    {
    sevseg.setChars("P01"); 
    //sevseg.refreshDisplay(); 
     
    }
    if (SetupProgramNumber==2) 
    {
    sevseg.setChars("P02"); 
    //sevseg.refreshDisplay(); 
   
    }
    if (SetupProgramNumber==3) 
    {
    sevseg.setChars("P03"); 
    //sevseg.refreshDisplay(); 
    } 
    if (SetupProgramNumber==4) 
    {
    sevseg.setChars("P04"); 
    //sevseg.refreshDisplay(); 
    } 
      if (SetupProgramNumber==5) 
    {
    sevseg.setChars("P05"); 
    //sevseg.refreshDisplay(); 
    } 
     if (SetupProgramNumber==6) 
    {
    sevseg.setChars("P06"); 
    //sevseg.refreshDisplay(); 
    } 

    if(SetupProgramNumber==7)
    {
    sevseg.setChars("P07"); 
    //sevseg.refreshDisplay();  
    }

    if(SetupProgramNumber==8)
    {
    sevseg.setChars("P08"); 
   // sevseg.refreshDisplay();  
    }

     if(SetupProgramNumber==9)
    {
    sevseg.setChars("P09"); 
   // sevseg.refreshDisplay();  
    }


    // displaying variables 
    if (SetupProgramNumber==10)    
    {
    sevseg.setNumberF(cutVoltage,1);
    
    //sevseg.refreshDisplay(); 
    } 
    if (SetupProgramNumber==11)    
    {
    sevseg.setNumberF(Setpoint,1);
   // sevseg.refreshDisplay(); 
    } 
    if (SetupProgramNumber==12)    
    {
    sevseg.setNumberF(Setpoint,1);
    //sevseg.refreshDisplay(); 
    } 
    if (SetupProgramNumber==13)    
    {
    sevseg.setNumber(SolarMaxPower);
   // sevseg.refreshDisplay(); 
    } 
    if (SetupProgramNumber==14)    
    {
    sevseg.setNumber(IncreaseTimeInSeconds);
   // sevseg.refreshDisplay(); 
    } 
     if (SetupProgramNumber==15)    
    {
    sevseg.setNumber(DecreaseTimeInSeconds);
   // sevseg.refreshDisplay(); 
    } 

    if (SetupProgramNumber==16)    
    {
    sevseg.setNumber(UtilityMaxPower);
   // sevseg.refreshDisplay(); 
    } 
      if (SetupProgramNumber==17)    
    {
    sevseg.setNumber(DelayTime);
   // sevseg.refreshDisplay(); 
    } 
    if (SetupProgramNumber==18)    
    {
    sevseg.setNumberF(VinBatteryError,1);
    //sevseg.refreshDisplay(); 
    } 
    if (SetupProgramNumber==19)    
    {

    if (contactorEnableLowBattery==1) sevseg.setChars("ON");
    if (contactorEnableLowBattery==0) sevseg.setChars("OFF");
   // sevseg.refreshDisplay(); 
    } 
 
    if (displayResetMessage==1)
    {
      sevseg.setChars("RST");
     // sevseg.refreshDisplay();
      esc++; 
      if (esc==2500)
      {
        displayResetMessage=0;
        esc=0;
      }
    }
//***************************************WElcome Screen**********************************************
    if (displayWelcomeScreen==1)
    {
      sevseg.setChars("SHC");
     // sevseg.refreshDisplay();
       esc++; 
      if (esc==1500)
      {
        displayWelcomeScreen=0;
        esc=0;
        displayVersionNumber=1;
      }
    }

     if (displayVersionNumber==1)
    {
      sevseg.setChars("V1.6");
      //sevseg.refreshDisplay();
       esc++; 
      if (esc==1500)
      {
        displayVersionNumber=0;
        esc=0;
      }
    }
// //------------------------------------------------END OF WELECOME SCREEN-----------------------------------------
  sevseg.refreshDisplay();            // refresh display according to variable selected 
 if (ScreenTimer > 9000) ScreenTimer=0; 

 }
//-------------------------------------------------------------------------------------------------------------------
// void PID_Compute()
// {

// if (digitalRead(AC_Available_Grid)==1) 
// {
// //-> for solar heating power 
// if(Vin_Battery_Calibrated>cutVoltage)
// {
 
// currentMillis = millis();
// if (currentMillis - previousMillis >= 1000)  // encrement variable every second 
// {
// previousMillis = currentMillis;
// SecondsReadTime++; 
// }
// if (SecondsReadTime>DelayTime) 
// {
// //How long since we last calculated
// now = millis();

// timeChange = (double)(now - lastTime);
// if (timeChange >= IncreaseTimeInSeconds*1000)
// {
//  // calculate error 
// PID_Error=((Vin_Battery_Calibrated+0.1)-(Setpoint))*amplifeError; // 0.1 is for making the shc start when float voltage is equal to the reading volt
//           //calculate the p value 
// PID_P=Kp*PID_Error; 
// if (PID_P <0) PID_P=0;
// if (PID_P > PIDMaxValue) PID_P=PIDMaxValue; 
// // calculate the I controller 
// PID_I=PID_I+ (Ki*PID_Error);
// if (PID_I <0) PID_I=0;
// if (PID_I > PIDMaxValue) PID_I=PIDMaxValue; 
// // calcaulte the pid value final 
// PID_Value=PID_P+PID_I ; 
// // to make range of pid 
// if (PID_Value <0) PID_Value=0;
// if (PID_Value > PIDMaxValue) PID_Value=PIDMaxValue; 
// //
// //Calculation method:
// //Solar Max Power : 40 % 
// //PID_maxValue= 255 (OCR1A max value) - ( 2.5 (step) * Solar Max Power)
// //OCR1A=map(PID_value(0-255),0,PID_maxValue,255,PID_maxValue+1 (+1 so OCR1A not become zero ever )) 
// //OCR1A as testing can be 260 or lower because according to equation i need 10ms 
// //Timer_count=  ( 10 MS * 10^-3 ) * ( 8*10^6 ) / 256 = 311
// //AS FROM TESTING :
// //best value was 260 or lower so load can be still on when the heating power is zero 
 
// HeatingPower=map(PID_Value,0,PIDMaxValue,0,SolarMaxPower); // map pid value show the range between 1- 260 what is the power 
// PWM_Value=map(PID_Value,0,PIDMaxValue,OCR1A_MaxValue,PID_MaxHeatingValue+1); // minus value of pwm is 1 and max value is 255 decfined in ocr1a_maxvalue
// lastTime=now;  // save last time for sampling time 
// } // end if sample 
// } // end if sample time 
// }  //end if vin_battery 

// else  if (Vin_Battery_Calibrated <= cutVoltage)
// {
//   PID_Value=0; 
//   PID_I=0; 
//   PID_P=0;
//   digitalWrite(PWM,LOW);
//   SecondsReadTime=0; 
//   PWM_Value=OCR1A_MaxValue;
//   TCCR1B=0x00 ; // stop the timer for no having any output so no output because i can't wait for interrupt to happen to turn off loads
//   HeatingPower=map(PID_Value,0,PIDMaxValue,0,SolarMaxPower);
//   PWM_Value=map(PID_Value,0,PIDMaxValue,OCR1A_MaxValue,PID_MaxHeatingValue+1); // minus value of pwm is 1 and max value is 260
//   // we also can stop timer to make output zero but i have done it in interrupts 
// }  // end else if 
// } // end if ac_available grid 


//  if(digitalRead(AC_Available_Grid)==0)
// {
//    currentMillis = millis();
//   if (currentMillis - previousMillis >= 1000)  // encrement variable every second 
//   {
//   previousMillis = currentMillis;
//   SecondsReadTime++; 
//   }

//   if (SecondsReadTime> DelayTime)  // check for the delay time required to start 
//   {
//     PID_ComputeForUtility();
//   }
// }
// } // end of pid_compute function
//---------------------------NEW PID COMPUTE FUNCTION BASED ON CHATGPT------------------------
void PID_Compute()
{
//Calculation method:
//Solar Max Power : 40 % 
//PID_maxValue= 245 (OCR1A max value) - ( 2.5 (step) * Solar Max Power)/100.0
//OCR1A=map(PID_value(0-255),0,PID_maxValue,255,PID_maxValue+1 (+1 so OCR1A not become zero ever )) 
//OCR1A as testing can be 260 or lower because according to equation i need 10ms 
//Timer_count=  ( 10 MS * 10^-3 ) * ( 8*10^6 ) / 256 = 311
//AS FROM TESTING :
//best value was 260 or lower so load can be still on when the heating power is zero 

    if (digitalRead(AC_Available_Grid) == 1) 
    {
        if (Vin_Battery_Calibrated > cutVoltage)
        {
            currentMillis = millis();

            if (currentMillis - previousMillis >= 1000)  // Increment variable every second
            {
                previousMillis = currentMillis;
                SecondsReadTime++;
            }

            if (SecondsReadTime > DelayTime) 
            {
                // Calculate PID only after delay
                now = millis();

                // Calculate the error
                PID_Error = ((Vin_Battery_Calibrated + 0.1) - Setpoint) * amplifeError;

                // Handle PID increase based on the error
                if (PID_Error > 0)  // Voltage is below setpoint, increase PWM_Value
                {
                    if ((now - lastIncreaseTime) >= IncreaseTimeInSeconds * 1000)
                    {
                        lastIncreaseTime = now;

                        // Calculate P and I terms
                        PID_P = Kp * PID_Error;
                        if (PID_P < 0) PID_P = 0;
                        if (PID_P > PIDMaxValue) PID_P = PIDMaxValue;

                        PID_I = PID_I + (Ki * PID_Error);
                        if (PID_I < 0) PID_I = 0;
                        if (PID_I > PIDMaxValue) PID_I = PIDMaxValue;

                        // Final PID value
                        PID_Value = PID_P + PID_I;
                        if (PID_Value < 0) PID_Value = 0;
                        if (PID_Value > PIDMaxValue) PID_Value = PIDMaxValue;

                        // Map PID value to heating power
                        HeatingPower = map(PID_Value, 0, PIDMaxValue, 0, SolarMaxPower);
                        //PID_MaxHeatingValue MUST BE MAPPED IN RANGE OF 255 
                        PWM_Value = map(PID_Value, 0, PIDMaxValue, OCR1A_MaxValue, PID_MaxHeatingValue + 1);
                    }
                }
                else if (PID_Error < 0)  // Voltage is above setpoint, decrease PWM_Value
                {
                    if ((now - lastDecreaseTime) >= DecreaseTimeInSeconds * 1000)
                    {
                        lastDecreaseTime = now;

                        // Calculate P and I terms (similar logic as for increase)
                        PID_P = Kp * PID_Error;
                        if (PID_P < 0) PID_P = 0;
                        if (PID_P > PIDMaxValue) PID_P = PIDMaxValue;

                        PID_I = PID_I + (Ki * PID_Error);
                        if (PID_I < 0) PID_I = 0;
                        if (PID_I > PIDMaxValue) PID_I = PIDMaxValue;

                        // Final PID value
                        PID_Value = PID_P + PID_I;
                        if (PID_Value < 0) PID_Value = 0;
                        if (PID_Value > PIDMaxValue) PID_Value = PIDMaxValue;

                        // Map PID value to heating power
                        HeatingPower = map(PID_Value, 0, PIDMaxValue, 0, SolarMaxPower);
                        PWM_Value = map(PID_Value, 0, PIDMaxValue, OCR1A_MaxValue, PID_MaxHeatingValue + 1);
                    }
                }

                // Save the last time for sampling
                lastTime = now;
            }
        }
        else
        {
            // Reset if voltage goes below cut level
            PID_Value = 0;
            PID_I = 0;
            PID_P = 0;
            digitalWrite(PWM, LOW);
            SecondsReadTime = 0;
            PWM_Value = OCR1A_MaxValue;
            TCCR1B = 0x00;  // Stop the timer
            HeatingPower = map(PID_Value, 0, PIDMaxValue, 0, SolarMaxPower);
            PWM_Value = map(PID_Value, 0, PIDMaxValue, OCR1A_MaxValue, PID_MaxHeatingValue + 1);
        }
    }

    if (digitalRead(AC_Available_Grid) == 0)
    {
        currentMillis = millis();
        if (currentMillis - previousMillis >= 1000)  // Increment variable every second
        {
            previousMillis = currentMillis;
            SecondsReadTime++;
        }

        if (SecondsReadTime > DelayTime)  // Check for the delay time required to start
        {
            PID_ComputeForUtility();
        }
    }
}

//-------------------------------------------Timer Init---------------------------------------
void Timer_Init()
{
TCCR1A=0 ;  // zero timer 
TCCR1B=0;  // zero timer 
TCCR1B |= (1<<WGM12); 
TIMSK1  |= (1 <<OCIE1A) | (1<< TOIE1) ;   // TIMER OVERFLOW AND INTERRUPT ENABLE 
}
//----------------------------------------Setup Program------------------------------
void SetupProgram()
{
insideSetup=1;
SetupProgramNumber=1; 
secondsFan=0 ;             // make time also zero 
fanState=0;               // make fan state zero 
digitalWrite(Fan,LOW);   // turn off fan
SetCutVoltage();
delay(500);
SetFloatVoltage();   // set point 
delay(500);
SetSolarMaxPower();
delay(500);
SetIncreaseTime();
delay(500);
SetDecreaseTime();
delay(500);
SetUtilityMaxPower();
delay(500);
SetDelayTime(); 
delay(500);
SetCalibrationVoltage();
delay(500); 
SetContactorLatch();
delay(500);
SetupProgramNumber=0; 
insideSetup=0;
}

//----------------------------------------Set Cut Voltage--------------------------------------
void SetCutVoltage()
{
delay(500);
while (digitalRead(Enter)==0 ) 
{
  SetupProgramNumber=1; 
}
delay(500);
while (digitalRead(Enter)==0 )
{
  SetupProgramNumber=10;
while (digitalRead(Up)==1 || digitalRead(Down)==1)
{
if (digitalRead(Up)==1) 
{
delay(150);  
cutVoltage+=0.1; 
}
if (digitalRead(Down)==1) 
{
delay(150);
cutVoltage-=0.1;
}
if(cutVoltage>60) cutVoltage=60.0;
if(cutVoltage<0)  cutVoltage=0;
} // end while up and down 
}  // end while enter 
EEPROM.put(0,cutVoltage);
} // end function 
//---------------------------------------Set Float Vooltage----------------------------------
void SetFloatVoltage()
{
delay(500);
while (digitalRead(Enter)==0) 
{
  SetupProgramNumber=2;

}
delay(500);
while (digitalRead(Enter)==0)
{
  SetupProgramNumber=12; 

while (digitalRead(Up)==1 || digitalRead(Down)==1)
{
if (digitalRead(Up)==1) 
{
delay(150);  
Setpoint+=0.1; 
}
if (digitalRead(Down)==1) 
{
delay(150);
Setpoint-=0.1;
}
} // end while up and down 
if(Setpoint>60.0) Setpoint=60.0;
if(Setpoint<0)  Setpoint=0;
}  // end while enter 
EEPROM.put(4,Setpoint);
}
//------------------------------------------Set Solar Max Power----------------------------------
void SetSolarMaxPower()
{
delay(500);
while(digitalRead(Enter)==0 )
{
  SetupProgramNumber=3;

} 
delay(500); 
while (digitalRead(Enter)==0 )
{
  SetupProgramNumber=13;

while (digitalRead(Up)==1 || digitalRead(Down)==1) 
{
if (digitalRead(Up)==1) 
{
delay(50);
SolarMaxPower++;
}
if (digitalRead(Down)==1) 
{
delay(50);
SolarMaxPower--;
}
if (SolarMaxPower>1000)  SolarMaxPower=1000;
if (SolarMaxPower<0) SolarMaxPower=0;
} // end while up and down
}  // end main while 
 //PID_MaxHeatingValue MUST BE MAPPED IN RANGE OF 255 
 //PID_MaxHeatingValue=OCR1A_MaxValue -  (OCR1A_MaxValue * SolarMaxPower) /100.0;  
PID_MaxHeatingValue=OCR1A_MaxValue -  (OCR1A_MaxValue * (SolarMaxPower/10.0)) /100.0;  
EEPROM.put(8,SolarMaxPower); 
EEPROM.put(90,PID_MaxHeatingValue); // for solar this value 
//-> must zero all values so the inverter want have a big ampers causing it to restart
PID_Value=0; 
PID_I=0; 
PID_P=0; 
PWM_Value=OCR1A_MaxValue;
}
//-----------------------------------------Sampling time-----------------------------------------
// for increasing 
void SetIncreaseTime()
{
delay(500);
 while(digitalRead(Enter)==0)
{
  SetupProgramNumber=4;

}
delay(500);
while (digitalRead(Enter)==0) 
{
  SetupProgramNumber=14;

while (digitalRead(Up)==1 || digitalRead(Down)==1) 
{
if (digitalRead(Up)==1) 
 {
delay(150);
//IncreaseTimeInSeconds++;
IncreaseTimeInSeconds++; 
}
if (digitalRead(Down)==1) 
{
delay(150);
IncreaseTimeInSeconds --;
} 
if(IncreaseTimeInSeconds > 60 ) IncreaseTimeInSeconds=60; 
if(IncreaseTimeInSeconds < 1 )  IncreaseTimeInSeconds=1; 
} // end while up and down
} // end main while 
EEPROM.write(10,IncreaseTimeInSeconds); 
}
//-----------------------------------------SET DECREASE TIME-----------------------------------
void SetDecreaseTime()
{
 delay(500);
 while(digitalRead(Enter)==0)
{
  SetupProgramNumber=5;

}
delay(500);
while (digitalRead(Enter)==0) 
{
  SetupProgramNumber=15;

while (digitalRead(Up)==1 || digitalRead(Down)==1) 
{
if (digitalRead(Up)==1) 
 {
delay(150);
DecreaseTimeInSeconds++; 
}
if (digitalRead(Down)==1) 
{
delay(150);
DecreaseTimeInSeconds --;
} 
if(DecreaseTimeInSeconds > 60 ) DecreaseTimeInSeconds=60; 
if(DecreaseTimeInSeconds < 1 )  DecreaseTimeInSeconds=1; 
} // end while up and down
} // end main while 
EEPROM.write(1,DecreaseTimeInSeconds);  
}

//-----------------------------------------Set utility max power-------------------------------
void SetUtilityMaxPower()
{
delay(500);
while(digitalRead(Enter)==0 )
{
SetupProgramNumber=6;

} 
delay(500); 
while (digitalRead(Enter)==0 )
{
SetupProgramNumber=16;

while (digitalRead(Up)==1 || digitalRead(Down)==1) 
{
if (digitalRead(Up)==1) 
{
delay(50);
UtilityMaxPower++;
}
if (digitalRead(Down)==1) 
{
delay(50);
UtilityMaxPower--;
}
if (UtilityMaxPower>1000)  UtilityMaxPower=1000;
if (UtilityMaxPower<0) UtilityMaxPower=0;
} // end while up and down
}  // end main while 
 //PID_MaxHeatingValueUtility MUST BE MAPPED IN RANGE OF 255 
 //PID_MaxHeatingValueUtility=OCR1A_MaxValue - ( OCR1A_MaxValue * UtilityMaxPower) /100.0;  // (2.5 = 255 / 100 )
PID_MaxHeatingValueUtility=OCR1A_MaxValue - ( OCR1A_MaxValue * (UtilityMaxPower/10.0)) /100.0;  // (2.5 = 255 / 100 )
EEPROM.put(11,UtilityMaxPower); 
EEPROM.put(120,PID_MaxHeatingValueUtility); // for solar this value
PID_Value=0; 
PID_I=0; 
PID_P=0; 
PWM_Value=OCR1A_MaxValue;
}
//-----------------------------------------SetDelayTime----------------------------------------
void SetDelayTime()
{
delay(500);
while(digitalRead(Enter)==0 )
{
SetupProgramNumber=7;

} 
delay(500); 
while (digitalRead(Enter)==0 )
{
SetupProgramNumber=17;

while (digitalRead(Up)==1 || digitalRead(Down)==1) 
{
if (digitalRead(Up)==1  ) 
{
delay(100);
DelayTime++;
}
if (digitalRead(Down)==1 ) 
{
delay(100);
DelayTime--;
}
if (DelayTime>240)  DelayTime=0;
if (DelayTime<0) DelayTime=0;
} // end while up and down
}  // end main while 
EEPROM.put(13,DelayTime); 
SecondsReadTime=0; 
}

//----------------------------------------SET BATTERY VOLTAGE CALIBRATION-----------------------
void SetCalibrationVoltage()
{
delay(500) ; 
while(digitalRead(Enter)==0 )
{
SetupProgramNumber=8;
} 
Read_Battery();
delay(500);
VinBatteryError=Vin_Battery_Calibrated;
delay(500); 
while (digitalRead(Enter)==0 )
{
SetupProgramNumber=18;

while (digitalRead(Up)==1 || digitalRead(Down)==1) 
{
if (digitalRead(Up)==1) 
{
delay(150);
VinBatteryError+=0.1;
}
if (digitalRead(Down)==1) 
{
delay(150);
VinBatteryError-=0.1;
}
if (VinBatteryError > 70.0  ) VinBatteryError=70.0;
if (VinBatteryError<0) VinBatteryError=0;
if (VinBatteryError>=Vin_Battery)
{
 addError=1;    // add
}else 
{
 addError=0; // minus 
}
//if (VinBatteryError<Vin_Battery_) addError=0;    // minus
} // end while up and down
}  // end while 
delay(100);
//save to eeprom 
VinBatteryDifference=fabs(VinBatteryError - Vin_Battery);
EEPROM.write(15,addError);
delay(100);
EEPROM.put(16,VinBatteryDifference);
}  // end function 
//-----------------------------------------CONTACTOR LATCH WHEN BATTERY LOW---------------------
void SetContactorLatch()
{
delay(500);
while(digitalRead(Enter)==0 )
{
SetupProgramNumber=9;
} 
delay(500); 
while (digitalRead(Enter)==0 )
{
SetupProgramNumber=19;

while (digitalRead(Up)==1 || digitalRead(Down)==1) 
{
if (digitalRead(Up)==1) 
{
delay(100);
contactorEnableLowBattery=1;
}
if (digitalRead(Down)==1) 
{
delay(100);
contactorEnableLowBattery=0;
}
if (contactorEnableLowBattery>1)  contactorEnableLowBattery=1;
if (contactorEnableLowBattery<0) contactorEnableLowBattery=0;
} // end while up and down
}  // end main while 
EEPROM.write(20,contactorEnableLowBattery); 
}
//-----------------------------------------EEPROM Load------------------------------------------
void EEPROM_Load()
{
EEPROM.get(0,cutVoltage);
EEPROM.get(4,Setpoint); 
//EEPROM.get(90,PID_MaxHeatingValue);
EEPROM.get(8,SolarMaxPower);
EEPROM.get(90,PID_MaxHeatingValue);
IncreaseTimeInSeconds=EEPROM.read(10);
DecreaseTimeInSeconds=EEPROM.read(30);
//UtilityMaxPower=EEPROM.read(11);
EEPROM.get(11,UtilityMaxPower);
//PID_MaxHeatingValueUtility=EEPROM.read(120);
EEPROM.get(120,PID_MaxHeatingValueUtility);
EEPROM.get(13,DelayTime); 
addError=EEPROM.read(15); 
EEPROM.get(16,VinBatteryDifference);
contactorEnableLowBattery=EEPROM.read(20); 
}
//---------------------------------------CheckForParams-----------------------------------------
void CheckForParams()
{
if (cutVoltage<0 || cutVoltage>70 || isnan(cutVoltage) ) 
{
  if (SystemBatteryMode==12) cutVoltage=12.5;
  if (SystemBatteryMode==24) cutVoltage=25.0; 
  if (SystemBatteryMode==48) cutVoltage=50.0; 
  EEPROM.put(0,cutVoltage); 
  EEPROM_Load();
}

if (Setpoint<0 || Setpoint>70 || isnan(Setpoint) ) 
{
  if (SystemBatteryMode==12) Setpoint=14.0;
  if (SystemBatteryMode==24) Setpoint=27.4; 
  if (SystemBatteryMode==48) Setpoint=54.0;
  EEPROM.put(4,Setpoint); 
  EEPROM_Load();
}

if (SolarMaxPower<0 || SolarMaxPower>1000 || isnan(SolarMaxPower)) 
{
  SolarMaxPower=750;
  EEPROM.put(8,SolarMaxPower); 
  EEPROM_Load();
}
if (PID_MaxHeatingValue<0 || PID_MaxHeatingValue>OCR1A_MaxValue || isnan(PID_MaxHeatingValue)) 
{
  PID_MaxHeatingValue=OCR1A_MaxValue - ( OCR1A_MaxValue * (SolarMaxPower/10.0)) /100.0;  // (2.5 = 255 / 100 )
  EEPROM.put(90,PID_MaxHeatingValue); 
  EEPROM_Load();
}
if (IncreaseTimeInSeconds<0 || IncreaseTimeInSeconds>100 || isnan(IncreaseTimeInSeconds)) 
{
  IncreaseTimeInSeconds=1;
  EEPROM.write(10,IncreaseTimeInSeconds); 
  EEPROM_Load();
}

if (DecreaseTimeInSeconds<0 || DecreaseTimeInSeconds>100 || isnan(DecreaseTimeInSeconds)) 
{
  DecreaseTimeInSeconds=1;
  EEPROM.write(30,DecreaseTimeInSeconds); 
  EEPROM_Load();
}
if (UtilityMaxPower<0 || UtilityMaxPower>1000 || isnan(UtilityMaxPower)) 
{
  UtilityMaxPower=1000;
  EEPROM.put(11,UtilityMaxPower); 
  EEPROM_Load();
}

if (PID_MaxHeatingValueUtility<0 || PID_MaxHeatingValueUtility>OCR1A_MaxValue || isnan(PID_MaxHeatingValueUtility)) 
{
  PID_MaxHeatingValueUtility=OCR1A_MaxValue - (OCR1A_MaxValue * (UtilityMaxPower/10.0)) /100.0;  // (2.5 = 255 / 100 )
  EEPROM.put(120,PID_MaxHeatingValueUtility); 
  EEPROM_Load();
}

if (DelayTime<0 || DelayTime>=240 || isnan(DelayTime)) 
{
  DelayTime=10;
  EEPROM.put(13,DelayTime); 
  EEPROM_Load();
}

 if (addError<0 || addError>1 || isnan(addError)) 
{
  addError=1;
  EEPROM.write(15,addError); 
  EEPROM_Load();
}

if (VinBatteryDifference<0 || VinBatteryDifference>=70 || isnan(VinBatteryDifference)) 
{
  VinBatteryDifference=0;
  EEPROM.put(16,VinBatteryDifference); 
  EEPROM_Load();
}


 if (contactorEnableLowBattery<0 || contactorEnableLowBattery>1 || isnan(contactorEnableLowBattery)) 
{
  contactorEnableLowBattery=0;
  EEPROM.write(20,contactorEnableLowBattery); 
  EEPROM_Load();
}

}
//-------------------------------------------Check For Grid--------------------------------------
void PID_ComputeForUtility()
{
  /*How long since we last calculated*/
now = millis();
timeChange = (double)(now - lastTime);
if (timeChange >= (IncreaseTimeInSeconds*1000))
{
accelerationValueUtility=PIDMaxValue / timetoReachMax ; 
// when grid available just increment the heating power regarless of the battery we done
PID_Value+=accelerationValueUtility; 
if (PID_Value <0) PID_Value=0;
if (PID_Value > PIDMaxValue) PID_Value=PIDMaxValue;
HeatingPower=map(PID_Value,0,PIDMaxValue,0,UtilityMaxPower); // map pid value show the range between 1- 260 what is the power 
PWM_Value=map(PID_Value,0,PIDMaxValue,OCR1A_MaxValue,PID_MaxHeatingValueUtility+1); // minus value of pwm is 1 and max value is 260 
lastTime=now;  // save last time 
} // end if sample time 
}
//------------------------------------------Check For Grid-------------------------------------
void CheckForGrid()
{
if(digitalRead(AC_Available_Grid)==0 && LoadsAlreadySwitchOff==0)
{
LoadsAlreadySwitchOff=1; 
PID_Value=0; 
PID_I=0; 
PID_P=0;
TCCR1B=0x00; // turn off 
digitalWrite(PWM,LOW);
HeatingPower=0;
SecondsReadTime=0;
digitalWrite(Contactor,1);   // TURN ON CONTACTOR
} 
else if (digitalRead(AC_Available_Grid)==1 && LoadsAlreadySwitchOff==1 )
{
LoadsAlreadySwitchOff=0;  
PID_Value=0; 
PID_I=0; 
PID_P=0;
TCCR1B=0x00; // turn off 
digitalWrite(PWM,LOW);
HeatingPower=0;
SecondsReadTime=0;
if(Vin_Battery_Calibrated>cutVoltage)
{
digitalWrite(Contactor,0);  // TURN OFF CONTACTOR
}
}
}
//---------------------------------------Fan Turn Off------------------------------------------
void checkFan()
{ 

//----------------------------------STOP FAN----------------------------------------
if (PWM_Value>=OCR1A_MaxValue  )
{ 
fanState=0;                // heating is off make this variable off and turn fan after 60 seconds
currentMillisFan=millis();
if(currentMillisFan-previousMillisFan>=1000)
{
previousMillisFan=currentMillisFan;
secondsFan++ ; 
}
if (secondsFan>=fanTimeOff) 
{
  digitalWrite(Fan,LOW); //turn off fan after 60 seconds
  secondsFan=0;
  fanisOn=0;
}
}

//---------------------------------STARTS FAN---------------------------------
if (PWM_Value>0 && PWM_Value <OCR1A_MaxValue && HeatingPower >= 150)
 { 
  fanState=1; //heating is on so fan must turn on 
  currentMillisFan=millis();
  if(currentMillisFan-previousMillisFan>=1000) 
  {
  previousMillisFan=currentMillisFan;
  secondsFan++ ; 
  }

  if (secondsFan>=fanTimeOn)
  {
  digitalWrite(Fan,HIGH);  //turn on fan 
  fanisOn=1;
  secondsFan=0;
  }
 }
//-------------------------------------------------------------------------

}

//----------------------------------------Check Battery System Voltage---------------------------
void CheckSystemBatteryMode()
{
if      (Vin_Battery>= 35 && Vin_Battery <= 70) SystemBatteryMode=48;
else if (Vin_Battery>=18 && Vin_Battery <=32) SystemBatteryMode=24;
else if (Vin_Battery >=1 && Vin_Battery<= 17 ) SystemBatteryMode=12;
else if(Vin_Battery==0) SystemBatteryMode=24;
else SystemBatteryMode=24; // take it as default
}
//--------------------------------------Factory Settings--------------------------------------
void factorySettings()
{
if (digitalRead(Up)==1 && digitalRead(Down)==1) 
{
EEPROM_FactorySettings();
displayResetMessage=1;
}

}
//------------------------------------EEPROM Factory Settings----------------------------------
void EEPROM_FactorySettings()
{
if (SystemBatteryMode==12)
{
cutVoltage=12.5;
Setpoint=14.0;

}
else if (SystemBatteryMode==24)
{
cutVoltage=25.0;
Setpoint=27.4;
} 
else if (SystemBatteryMode==48)
{
cutVoltage=50;
Setpoint=54.0;
}
else SystemBatteryMode=24;
/* Global Varaiables */
SolarMaxPower=750;
PID_MaxHeatingValue=OCR1A_MaxValue -  (OCR1A_MaxValue * (SolarMaxPower/10.0)) /100.0;  // old equation was : PID_MaxHeatingValue=OCR1A_MaxValue - ( (OCR1A_MaxValue/100) * SolarMaxPower);
IncreaseTimeInSeconds=1;   //samples of pid controller taked snapshots 
DecreaseTimeInSeconds=1;
UtilityMaxPower=1000;     // max utility power 
PID_MaxHeatingValueUtility=OCR1A_MaxValue - ( OCR1A_MaxValue * (UtilityMaxPower/10.0)) /100.0;  // (2.5 = 255 / 100 )
DelayTime=10;   // delay time to start the heater
addError=1; 
VinBatteryDifference=0; 
contactorEnableLowBattery=0; 
/*ً Save Values to EEPROM */
EEPROM.put(0,cutVoltage); 
EEPROM.put(4,Setpoint); 
EEPROM.put(8,SolarMaxPower); 
EEPROM.put(90,PID_MaxHeatingValue);
EEPROM.write(10,IncreaseTimeInSeconds); 
EEPROM.write(30,DecreaseTimeInSeconds); 
EEPROM.put(11,UtilityMaxPower); 
EEPROM.put(120,PID_MaxHeatingValueUtility); 
EEPROM.put(13,DelayTime); 
EEPROM.write(15,addError);
EEPROM.put(16,VinBatteryDifference);
EEPROM.write(20,contactorEnableLowBattery); 
EEPROM_Load();
}
//-----------------------------------------PRESS DETECT-----------------------------------------
void Press_Detect()
{
InProgramMode=true; 
SetupProgram();
delay(500);
}
//---------------------------------------WELCOME SCREEN-----------------------------------------
void WelcomeScreen()
{
displayWelcomeScreen=1;
}

//-------------------------------------CHECK FOR LOW CUT OFF----------------------------------
void checkCutOffVoltage()
{
if(digitalRead(AC_Available_Grid)==1)
{

 if (Vin_Battery_Calibrated<=cutVoltage)
 {
  PID_Value=0; 
  PID_I=0; 
  PID_P=0;
  digitalWrite(PWM,LOW) ; 
  TCCR1B=0x00;
 }

}

}
//--------------------------------------CONTACTOR STATE WHEN BATTERY LOW------------------------
//when battery is low to make sure that output is switched off out of inverter the contactor will turn to another mode 
// so on. when heating is started again the contactor will return to basic state 
// user must enable program 8
void checkContactor()
{
  
  if (PWM_Value<OCR1A_MaxValue && digitalRead(AC_Available_Grid)==1 && contactorEnableLowBattery==1) 
  {
    digitalWrite(Contactor,LOW);
  }
  else if (Vin_Battery_Calibrated<=cutVoltage && digitalRead(AC_Available_Grid)==1 && contactorEnableLowBattery==1)
  {
     digitalWrite(Contactor,HIGH); // turn off the loads from no to nc for battery pro
  }
}
//---------------------------------------------------SCREEN--------------------------------------------------
void Screen()
{
    if (ScreenTimer> 0 && ScreenTimer < 5000 && insideSetup==0 && SetupProgramNumber==0 && displayResetMessage==0 &&  displayWelcomeScreen==0 &&  displayVersionNumber==0 
    )
    {
     sevseg.setNumberF(Vin_Battery_Calibrated,1); // Displays '3.141'
  
    
    }
    if (ScreenTimer>5000 && ScreenTimer< 7000 && insideSetup==0 && SetupProgramNumber==0 && displayResetMessage==0 &&  displayWelcomeScreen==0 &&  displayVersionNumber==0 
    ) 
    {
     sevseg.setNumber(HeatingPower); 
    //sevseg.setNumberF(Setpoint,1); // Displays '3.141'
     }  
      if (ScreenTimer>7000 && ScreenTimer< 9000 && insideSetup==0 && SetupProgramNumber==0 && displayResetMessage==0 &&  displayWelcomeScreen==0 &&  displayVersionNumber==0 
   ) 
    {
      if (digitalRead(AC_Available_Grid)==0)
      {
      sevseg.setChars("UON");
      
      } else 
      {
       sevseg.setNumberF(Vin_Battery_Calibrated,1); // Displays '3.141'
       }
    }  
}
//*****************************************MAIN LOOP********************************************
void setup() {
  // put your setup code here, to run once:
Segment_Init();
WelcomeScreen();
Segment_Timer_Update();
GPIO_Init();  
EEPROM_Load();
Timer_Init();   
}
//-> start developing
void loop() {
  // put your main code here, to run repeatedly:
   button.tick();
   CheckForParams();
   Read_Battery();      
   PID_Compute();
   CheckForGrid();   
   checkFan();
   checkContactor(); 
   CheckSystemBatteryMode();   // to determine battery system mode 
   factorySettings();
   checkCutOffVoltage();      // to make sure all is off when battery voltage is down
   Screen(); 
   delay(100);
}   // end of main ... 