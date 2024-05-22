#include <Arduino.h>
#include "hexapodHead.h"

#define DEBUG_HEAD


hexapodHead::hexapodHead() : panPIDLoop(40, 0, 40, true, 0, 180) , tiltPIDLoop(50, 0, 50, true, 40, 160), lineFollowPIDLoop(40, 0, 40, false, -20, 20){
  panCenterOffset = 0;
  tiltCenterOffset = -8;
  panMin = 0;
  panMax = 180;
  tiltMin = 40;
  tiltMax = 160;
  panAngle = 90;
  tiltAngle = 90;

  panDeviation = 0;
  tiltDeviation = 0;
  lineFollowDeviation = 0;
}

void hexapodHead::begin(uint8_t _panServoID, uint8_t _tiltServoID){
  panServoID = _panServoID;
  tiltServoID = _tiltServoID;
  errorCode = pixy.init();
  SERVO.pSerial = &Serial2;
  sensor.init(0x29);
  oled2.begin();
  oled2.setPowerSave(0);
  oled2.setFont(u8x8_font_chroma48medium8_r);
  oled2.setFlipMode(1);
  //PIDLoop panPIDLoop(40, 0, 40, true, 0, 180);
  //PIDLoop tiltPIDLoop(50, 0, 50, true, 40, 160);

  oled2.setCursor(0, 4);
  oled2.print(errorCode);

  //pixy.init();
  pixy.setLamp(254, 0);



}



void hexapodHead::scanCCC(){
  // grab blocks!
  pixy.ccc.getBlocks(true, 1,1);

  // If there are detect blocks, print them!
  if (pixy.ccc.numBlocks)
  {
    panDeviation = ((pixy.frameWidth/2)-pixy.ccc.blocks[0].m_x)*-1;
    tiltDeviation = ((pixy.frameHeight/2)-pixy.ccc.blocks[0].m_y)*-1;

    #ifdef DEBUG_HEAD
    oled2.clearLine(2);
    oled2.setCursor(0, 2);
    oled2.print("X:");
    oled2.setCursor(3, 2);
    oled2.print(pixy.ccc.blocks[0].m_x);
    oled2.clearLine(3);
    oled2.setCursor(0, 3);
    oled2.print("Y:");
    oled2.setCursor(3, 3);
    oled2.print(pixy.ccc.blocks[0].m_y);
    #endif


    panPIDLoop.update(panDeviation);
    tiltPIDLoop.update(tiltDeviation);

    #ifdef DEBUG_HEAD
    oled2.setCursor(8, 2);
    oled2.print(panDeviation);
    oled2.setCursor(8, 3);
    oled2.print(tiltDeviation);
    #endif

    panAngle = panPIDLoop.m_command;
    tiltAngle = tiltPIDLoop.m_command;

  }
  else
  {
    panPIDLoop.reset();
    tiltPIDLoop.reset();
    panAngle = panPIDLoop.m_command;
    tiltAngle = tiltPIDLoop.m_command;
  }
  #ifdef DEBUG_HEAD
  oled2.clearLine(4);
  oled2.setCursor(8, 4);
  oled2.print(panPIDLoop.m_command);
  oled2.clearLine(5);
  oled2.setCursor(8, 5);
  oled2.print(panPIDLoop.m_command);
  #endif
}

void hexapodHead::scanLine(){
  pixy.line.getMainFeatures();
  if (pixy.line.numVectors)
  pixy.line.vectors->print();
  if (pixy.line.numIntersections)
  pixy.line.intersections->print();
  if (pixy.line.barcodes)
  pixy.line.barcodes->print();
}

void hexapodHead::scanLineSingle(){
  pixy.line.getMainFeatures();
  if (pixy.line.numVectors){
    //Serial.printf("Index %3i Kopf %3i Hintern %3i ||", pixy.line.vectors->m_index, ((pixy.frameWidth/2)-pixy.line.vectors->m_x1)*-1, ((pixy.frameWidth/2)-pixy.line.vectors->m_x0)*-1);
    //Serial.printf(" Höhe %3i Breite %3i Weite %3i \n",pixy.frameHeight,pixy.frameWidth, (pixy.frameHeight/2)-pixy.line.vectors->m_y1);
    //lineFollowDeviation = ((pixy.frameWidth/2)-pixy.line.vectors->m_x1)*-1;
    //ineFollowWide = (pixy.frameHeight/2)-pixy.line.vectors->m_y1;

    lineFollowDeviation = (((79/2)-pixy.line.vectors->m_x1)/2)*-1;
    lineFollowWide = (52/2)-pixy.line.vectors->m_y1;

    //Serial.printf("Winkel = %3i Wie weit = %3i \n",lineFollowDeviation,lineFollowWide);


    #ifdef DEBUG_HEAD
    if (lineFollowDeviation < -15 || lineFollowDeviation > 15 || lineFollowWide < -10 || lineFollowWide > 10){
      oled2.clearLine(4);
      oled2.setCursor(6, 4);
      oled2.print("Line Error");
    }
    #endif

    if (lineFollowDeviation < -10){
      lineFollowDeviation = -10;
    }
    if (lineFollowDeviation > 10){
      lineFollowDeviation = 10;
    }
    if (lineFollowWide < -15){
      lineFollowWide = -15;
    }
    if (lineFollowWide > 15){
      lineFollowWide = 15;
    }


  }
  else{
    lineFollowDeviation = 0;
    lineFollowWide = 0;
  }

  #ifdef DEBUG_HEAD
  oled2.clearLine(6);
  oled2.setCursor(0, 6);
  oled2.print(lineFollowDeviation);
  oled2.setCursor(5, 6);
  oled2.print(lineFollowWide);
  //Serial.printf("Winkel = %3i Wie weit = %3i \n",lineFollowDeviation,lineFollowWide);
  #endif

  //delay(2000);
  /*
  uint8_t m_x0;
  uint8_t m_y0;
  uint8_t m_x1;
  uint8_t m_y1;
  uint8_t m_index;
  uint8_t m_flags;
  */

}

void hexapodHead::scanLineAll(){
  int8_t i;
  char buf[128];

  pixy.line.getAllFeatures();

  // print all vectors
  for (i=0; i<pixy.line.numVectors; i++)
  {
    sprintf(buf, "line %d: ", i);
    Serial.print(buf);
    pixy.line.vectors[i].print();
  }

  // print all intersections
  for (i=0; i<pixy.line.numIntersections; i++)
  {
    sprintf(buf, "intersection %d: ", i);
    Serial.print(buf);
    pixy.line.intersections[i].print();
  }

  // print all barcodes
  for (i=0; i<pixy.line.numBarcodes; i++)
  {
    sprintf(buf, "barcode %d: ", i);
    Serial.print(buf);
    pixy.line.barcodes[i].print();
  }
}

void hexapodHead::FollowCCC(){
  if ((panAngle+panCenterOffset) < panMin) panAngle=panMin;
  if ((panAngle+panCenterOffset) > panMax) panAngle=panMax;
  if ((tiltAngle+tiltCenterOffset) < tiltMin) tiltAngle=tiltMin;
  if ((tiltAngle+tiltCenterOffset) > tiltMax) tiltAngle=tiltMax;
  SERVO.WritePos(panServoID, angleMap(panAngle+panCenterOffset), 0, 250);
  SERVO.WritePos(tiltServoID, angleMap(tiltAngle+tiltCenterOffset), 0, 250);
}

void hexapodHead::headMoveAbsolut(int _pan, int _tilt){
  #ifdef DEBUG_HEAD
  oled2.setCursor(0, 6);
  oled2.print(panServoID);
  #endif
  panNow = _pan;
  tiltNow = _tilt;
  SERVO.WritePos(panServoID, angleMap(_pan), 0, 2500); //pan
  SERVO.WritePos(tiltServoID, angleMap(_tilt), 0, 2500); //tilt
}

void hexapodHead::headMoveRelative(int _pan, int _tilt){
  #ifdef DEBUG_HEAD
  oled2.setCursor(0, 6);
  oled2.print(panServoID);
  #endif
  panNow = panNow + _pan;
  tiltNow = tiltNow + _tilt;
  SERVO.WritePos(panServoID, angleMap(panNow), 100, 0); //pan
  SERVO.WritePos(tiltServoID, angleMap(tiltNow), 100, 0); //tilt
}

void hexapodHead::beginFollowLine(){
  SERVO.WritePos(panServoID, angleMap(90), 0, 2500); //pan
  SERVO.WritePos(tiltServoID, angleMap(150), 0, 2500); //tilt
  pixy.line.getMainFeatures();

  delay(2000);
  #ifdef DEBUG_HEAD
  oled2.setCursor(0, 6);
  oled2.print(pixy.line.numVectors);
  delay(4000);
  #endif
}

int hexapodHead::getDistance(){
  int distance = sensor.readRangeSingleMillimeters();
  #ifdef DEBUG_HEAD
  oled2.clearLine(4);
  oled2.setCursor(6, 4);
  oled2.print(distance);
  #endif
  return distance;
}

void hexapodHead::test_leds(int ms)
{
  #ifdef DEBUG_HEAD
  Serial.println("Test LEDs!");
  oled2.setCursor(0, 1);
  oled2.print("Test LEDs!");
  #endif
  pixy.setLamp(254, 0);
  delay(ms);
  pixy.setLamp(0, 0);
  delay(ms);
}

int hexapodHead::angleMap(int angle){
  int position=511;
  position = map(angle,0,180,210,845);
  if (position < 0)position =0;
  if (position > 1023)position =1023;
  return position;
}
