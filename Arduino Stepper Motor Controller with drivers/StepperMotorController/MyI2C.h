/*
 *  MyI2C.h - Library for I2C bus. 
 *  Recreated by Zbigniew Barczyk 5 April 2022
 */
 #ifndef MyI2C_h
 #define MyI2C_h

 #include "Arduino.h"

 class MyI2C
 {
  public:
    MyI2C(int SCL, int SDA, int Addr);
    void write1Message(unsigned char addr, char data);
    void clockwise(int motorspeed);
    void antiClockwise(int motorspeed);
    void writeCharArray(int address, char str[], char str2[], int word1len, int word2len);
    void lcdClear();
    void functionSet();
    void lcdOn();
    void lcdOff();
    void lcdHome();
    void Line2();
    void Line1();
    void writeCmd(unsigned char x);
    void writeData(unsigned char x);
    

  private:
    void stopSignal();
    void startSignal();
    unsigned char readAck();
    void write1Byte(char value);
  
    int _pinSDA;
    int _pinSCL;
    int _addr;
 };
 #endif
