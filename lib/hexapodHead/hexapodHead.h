#ifndef _hexapodHead_h
#define _hexapodHead_h

#include <Arduino.h>
#include <U8x8lib.h>
#include "VL53L0X.h"
#include <SCServo.h>
#include <Pixy2UART.h>
#include "PIDLoop.h"

//#define PID_MAX_INTEGRAL         2000
//#define ZUMO_BASE_DEADBAND       20

#define DEBUG_HEAD


class hexapodHead {
public:

  hexapodHead();
  void begin(uint8_t panServoID, uint8_t tiltServoID);
  PIDLoop panPIDLoop;
  PIDLoop tiltPIDLoop;
  PIDLoop lineFollowPIDLoop;
  //PIDLoop panPIDLoop(int 40, int 0, int 40, bool true, int 0, int 180);
  //PIDLoop tiltPIDLoop(int 50, int 0, int 50, bool true, int 40, int 160);

  void scanCCC();
  void scanLine();
  void scanLineSingle();
  void scanLineAll();

  void FollowCCC();
  void headMoveAbsolut(int pan, int tilt);
  void headMoveRelative(int pan, int tilt);
  void beginFollowLine();
  int getDistance();
  void test_leds(int ms);

  int panDeviation;
  int tiltDeviation;
  int lineFollowDeviation;
  int lineFollowWide;

private:
  U8X8_SSD1306_128X64_NONAME_HW_I2C oled2;
  SCSCL SERVO;
  VL53L0X sensor;
  Pixy2UART pixy;

  int angleMap(int angle);

  uint8_t panServoID;
  uint8_t tiltServoID;
  int panCenterOffset;
  int tiltCenterOffset;
  int panMin;
  int panMax;
  int panNow;
  int tiltMin;
  int tiltMax;
  int tiltNow;
  int panAngle;
  int tiltAngle;
  int errorCode;


  //




};





#endif
