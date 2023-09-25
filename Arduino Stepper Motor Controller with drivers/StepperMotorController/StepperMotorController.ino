#include "MyI2C.h"

int mySDA = 10;
int mySCL = 9;
int I2C2Add = 0b01110000; //0b0111 A2 A1 A0 R/N
int I2C1Add = 0b01110010; //0b0111 A2 A1 A0 R/N

//  now I2C2Add RS = 0b00000100, RW = 0b00000010, EN = 0b00000001

MyI2C i2c1(mySCL, mySDA, I2C1Add);
MyI2C i2c2(mySCL, mySDA, I2C2Add);

int motorSpeed = 1000;    //variable to set stepper speed
int count = 0;            //count of steps made
int countsperrev = 100;   //number of steps per full revolution, 512 default

void setup() {

    //initialize
    delayMicroseconds(16000);
    i2c2.functionSet();
    
    delayMicroseconds(4500);
    i2c2.functionSet();
    
    delayMicroseconds(150);
    i2c2.functionSet();
    i2c2.lcdOff();
    i2c2.lcdClear();
    i2c2.lcdOn();
}

void loop() {
i2c2.lcdHome();

if(count == 1)
{
  if(motorSpeed == 1000)
  {
    i2c2.writeCharArray(I2C2Add, "clockwise", "fast", 9, 4);
  }
  else if(motorSpeed == 3000)
  {
    i2c2.writeCharArray(I2C2Add, "clockwise", "medium", 9, 6);
  }
  else if(motorSpeed == 5000)
  {
    i2c2.writeCharArray(I2C2Add, "clockwise", "slow", 9, 4);
  }
  else if(motorSpeed == 7000)
  {
      motorSpeed = 1000;
  }
}
else if(count == countsperrev)
{
if(motorSpeed == 1000)
  {
    i2c2.writeCharArray(I2C2Add, "anticlockwise", "fast", 13, 4);
  }
  else if(motorSpeed == 3000)
  {
    i2c2.writeCharArray(I2C2Add, "anticlockwise", "medium", 13, 6);
  }
  else if(motorSpeed == 5000)
  {
    i2c2.writeCharArray(I2C2Add, "anticlockwise", "slow", 13, 4);
  }
  else if(motorSpeed == 7000)
  {
      motorSpeed = 1000;
  }
}

if(count == countsperrev * 2)
{
  motorSpeed +=2000;
}

if(count < countsperrev)
  i2c2.clockwise(motorSpeed);
else if(count == countsperrev * 2)
    count = 0;
else
  i2c2.antiClockwise(motorSpeed);
count++;


}
