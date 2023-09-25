#include <PCF8574.h>

#define SDA A4   //analog  I2C chip
#define SCL A5   //analog I2C chip
#define Shift_Latch 4 //digital shift reg
#define Shift_Data 5    //digital shift reg
#define Shift_Clock 6 //digital shift reg
#define Dis1 8  //n1 led gnd
#define Dis2 9  //n2 led gnd
#define Dis3 10 //n3 led gnd    //for extra digit, unused
#define Dis4 11 //n4 led gnd    //for extra digit, unused
#define ButtonPin 2

const int segDisp[10] = {63, 6, 91, 79, 102, 109, 125, 7, 127, 111};
const int servDelay[10] = {2000, 1800, 1600, 1400, 1200, 1000, 800, 600, 400, 200};

PCF8574 PCF_01(0x38);
unsigned long nextMillis;
unsigned long currentMillis;
volatile unsigned long debounceTimer;
volatile unsigned long lastDebounceTimer = 0;
int timer;
boolean counting = false;



void setup() {
  
pinMode(SDA, OUTPUT);
pinMode(SCL, OUTPUT);
pinMode(Shift_Latch, OUTPUT);
pinMode(Shift_Data, OUTPUT);
pinMode(Shift_Clock, OUTPUT);
pinMode(Dis1, OUTPUT);
pinMode(Dis2, OUTPUT);
pinMode(Dis3, OUTPUT);
pinMode(Dis4, OUTPUT);
pinMode(ButtonPin, INPUT_PULLUP);
Serial.begin(9600);
PCF_01.begin();
int millis;
attachInterrupt(digitalPinToInterrupt(ButtonPin), buttonPress, CHANGE);

     digitalWrite(Dis1, HIGH);
     digitalWrite(Dis2, HIGH);
     digitalWrite(Dis3, HIGH);
     digitalWrite(Dis4, HIGH);
     timer = 0;

}

void loop()
{
  
  
  currentMillis = millis();
  
  if(counting == true)    //are we counting?
  {
    if(currentMillis >= nextMillis) //clock
    {
      nextMillis = currentMillis + 1000;
      if(timer<99)
        timer=timer+1;
      else
        timer=0;
    }
  }
  
  displayTime(timer);   //updates servo and displays 7 segment display value
  
    
}


void allDigit(int a, int b)   //void allDigit(int a, int b, int c, int d) //4 digit version //mothod for controlling display, for 2 also for 4 
{
   digitalWrite(Shift_Latch, LOW);
    shiftOut(Shift_Data, Shift_Clock, MSBFIRST, segDisp[a]);
    digitalWrite(Shift_Latch, HIGH);
    digitalWrite(Dis1, LOW);
    digitalWrite(Dis1, HIGH);
    
    digitalWrite(Shift_Latch, LOW);
    shiftOut(Shift_Data, Shift_Clock, MSBFIRST, segDisp[b]);
    digitalWrite(Shift_Latch, HIGH);
    digitalWrite(Dis2, LOW);
    digitalWrite(Dis2, HIGH);

//commentd out because time goes to 99 only, kept in case that changes
//    digitalWrite(Shift_Latch, LOW);
//    shiftOut(Shift_Data, Shift_Clock, MSBFIRST, c);
//    digitalWrite(Shift_Latch, HIGH);
//    digitalWrite(Dis3, LOW);
//    digitalWrite(Dis3, HIGH);
//
//
//    digitalWrite(Shift_Latch, LOW);
//    shiftOut(Shift_Data, Shift_Clock, MSBFIRST, d);
//    digitalWrite(Shift_Latch, HIGH);
//    digitalWrite(Dis4, LOW);
//    digitalWrite(Dis4, HIGH);
}
void allMotors(int a, int b)    //method for controlling both servos
{
      PCF_01.write(0, HIGH);
      delayMicroseconds(servDelay[a]);
      PCF_01.write(0, LOW);
      delayMicroseconds(20000-servDelay[a]); 

      PCF_01.write(1, HIGH);
      delayMicroseconds(servDelay[b]);
      PCF_01.write(1, LOW);
      delayMicroseconds(20000-servDelay[b]); 
}

void buttonPress()  //interrupt method for button debounce
{ 
  debounceTimer = millis();
  if((debounceTimer - lastDebounceTimer)>= 2000 && digitalRead(ButtonPin) == 1)
  {
        timer=0;
        counting = false;
  }
  else if((debounceTimer - lastDebounceTimer)>= 200 && digitalRead(ButtonPin) == 1)
  {
        counting = !counting;
  }
  lastDebounceTimer = debounceTimer;
}

void displayTime(int timeToDisplay)   //time can be between 0 and 99 //method for combining servo and display
{
  allDigit(timeToDisplay/10, timeToDisplay%10);
  allMotors(timeToDisplay/10, timeToDisplay%10);
}
