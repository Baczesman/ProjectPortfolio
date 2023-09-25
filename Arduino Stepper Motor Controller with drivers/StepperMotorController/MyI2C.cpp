
/*
 *  MyI2C.cpp - Library for I2C bus. 
 *  Recreated by Zbigniew Barczyk 5 April 2022
 *  
 */

 #include "Arduino.h"
 #include "MyI2C.h"

 MyI2C::MyI2C(int SCL, int SDA, int Addr)
 {
  pinMode(SCL, OUTPUT);
  pinMode(SDA, OUTPUT);

  _pinSCL = SCL;
  _pinSDA = SDA;
  _addr = Addr;
  
 }

void MyI2C::startSignal(){    //signal to start sending data
  pinMode(_pinSCL, OUTPUT);
  pinMode(_pinSDA, OUTPUT);
  digitalWrite(_pinSDA, LOW);
  delayMicroseconds(10); //min 4us, nut not matter too much, 1 also works?? huh?
  digitalWrite(_pinSCL, LOW);
}

void MyI2C::stopSignal(){   //signal to stop sending data
  pinMode(_pinSCL, OUTPUT);
  pinMode(_pinSDA, OUTPUT);
  digitalWrite(_pinSCL, HIGH);
  delayMicroseconds(10); //min 4us, nut not matter too much, 1 also works?? huh?
  digitalWrite(_pinSDA, HIGH);
}

unsigned char MyI2C::readAck(){ //function to read acknowledgement from I2c
  int ack = HIGH;
  pinMode(_pinSDA, INPUT);
  digitalWrite(_pinSCL, HIGH);
  delayMicroseconds(1);
  ack = digitalRead(_pinSDA);
  digitalWrite(_pinSCL, LOW);
  pinMode(_pinSDA, OUTPUT);
  return ack;
}

void MyI2C::write1Byte(char value){ //writes 1 byte by bit, doesnt have address or acknowledgement
  pinMode(_pinSCL, OUTPUT);
  pinMode(_pinSDA, OUTPUT);
  //write value. MSB first
  for(int i=0; i<8; i++){

    if((value & (0x80 >> i)) == 0) {
      digitalWrite(_pinSDA, LOW);
    } else{
      digitalWrite(_pinSDA, HIGH);
    }
    delayMicroseconds(1);
    digitalWrite(_pinSCL, HIGH);
    delayMicroseconds(4);
    digitalWrite(_pinSCL, LOW);
  }
}

void MyI2C::write1Message(unsigned char addr, char data){ //sends an 8 bit to the correct I2c using write1byte and timigs

  int ack = HIGH;

  startSignal();

  pinMode(_pinSDA, OUTPUT);
  pinMode(_pinSCL, OUTPUT);
  
  //write adddress
  write1Byte(addr);
  
  ack = readAck();
  //do sth if ack = high
  
  //write data
  write1Byte(data);
  
  delayMicroseconds(4);

  ack = readAck();
  //do sth if ack = high

  stopSignal();

}





void MyI2C::clockwise(int motorSpeed) //moves the stepper motor clockwise (higher motorSpeed - lover motor speed)
{
  write1Message(0b01110000, 0b10010000);
    delayMicroseconds(motorSpeed);
  write1Message(0b01110000, 0b00010000);
    delayMicroseconds(motorSpeed);
  write1Message(0b01110000, 0b00110000);
    delayMicroseconds(motorSpeed);
  write1Message(0b01110000, 0b00100000);
    delayMicroseconds(motorSpeed);
  write1Message(0b01110000, 0b01100000);
    delayMicroseconds(motorSpeed);
  write1Message(0b01110000, 0b01000000);
    delayMicroseconds(motorSpeed);
  write1Message(0b01110000, 0b11000000);
    delayMicroseconds(motorSpeed);
  write1Message(0b01110000, 0b10000000);
    delayMicroseconds(motorSpeed);
}

void MyI2C::antiClockwise(int motorSpeed) //moves the stepper motor anticlockwise (higher motorSpeed - lover motor speed)
{
  write1Message(0b01110000, 0b10000000);
    delayMicroseconds(motorSpeed);
  write1Message(0b01110000, 0b11000000);
    delayMicroseconds(motorSpeed);
  write1Message(0b01110000, 0b01000000);
    delayMicroseconds(motorSpeed);
  write1Message(0b01110000, 0b0110000);
    delayMicroseconds(motorSpeed);
  write1Message(0b01110000, 0b00100000);
    delayMicroseconds(motorSpeed);
  write1Message(0b01110000, 0b00110000);
    delayMicroseconds(motorSpeed);
  write1Message(0b01110000, 0b00010000);
    delayMicroseconds(motorSpeed);
  write1Message(0b01110000, 0b10010000);
    delayMicroseconds(motorSpeed);
}


void MyI2C::lcdClear()  //clears the display
{
  writeCmd(0x01);
  delayMicroseconds(2000);
}
void MyI2C::functionSet() //sets display mode(used in setup)
{
  writeCmd(0x38);
}
void MyI2C::lcdOn() //turns the display on (used in setup)
{
  writeCmd(0b00001100);
  delayMicroseconds(2000);
}
void MyI2C::lcdOff()  //turns display off
{
  writeCmd(0b00001000);
  delayMicroseconds(2000);
}
void MyI2C::lcdHome() //return to position 0 on the lcd display
{
  writeCmd(0x02);
  delayMicroseconds(2000);
}
void MyI2C::Line2() // returns to first position of econd line
{
  writeCmd(0b11000000);
  delayMicroseconds(2000);
}
void MyI2C::Line1() //returns to first position of firt line
{
  writeCmd(0b10000000);
  delayMicroseconds(2000);
}


void MyI2C::writeCharArray(int address, char str[], char str2[], int word1len, int word2len)  //writes the first word and second word in first row and second row, also needs length of both words
{
  lcdClear();
  lcdHome();
  Line1();
    for(int i=0; i<word1len; i++)  //does not work for some reason sizeof is 2 not 12
      writeData(str[i]);  
  Line2();
    for(int i=0; i<word2len; i++)  //does not work for some reason sizeof is 2 not 12
      writeData(str2[i]);  
}

void MyI2C::writeCmd(unsigned char x) //contains the timings to give lcd commands
{
  write1Message(0b01110000, 0b00000000);
  delayMicroseconds(5);

  write1Message(0b01110000, 0b00000000);
  write1Message(0b01110000, 0b00000000);
  write1Message(0b01110000, 0b00000001);
  delayMicroseconds(5);

    write1Message(0b01110010, x);

  write1Message(0b01110000, 0b00000000);
  delayMicroseconds(100);
}

void MyI2C::writeData(unsigned char x)    //contains timings to give lcd text to dispay
{
  write1Message(0b01110000, 0b00000000);
  delayMicroseconds(5);

  write1Message(0b01110000, 0b00000100);
  write1Message(0b01110000, 0b00000100);
  write1Message(0b01110000, 0b00000101);
  delayMicroseconds(5);

  write1Message(0b01110010, x);

  write1Message(0b01110000, 0b00000100);
  delayMicroseconds(100);
}
