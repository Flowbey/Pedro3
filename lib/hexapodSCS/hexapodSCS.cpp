#include <Arduino.h>
#include "hexapodSCS.h"
#include <math.h>




hexapodSCS::hexapodSCS() {

}

void hexapodSCS::begin() {
  Serial2.begin(1000000, SERIAL_8N1, RXD2, TXD2);
  SCservo.pSerial = &Serial2;
  delay(500);
  SCservo.EnableTorque(1, 1);
  SCservo.EnableTorque(2, 1);



}



//======================================
//====================================== Attach servo
//======================================

void hexapodSCS::attachServo( uint8_t servoNumber, uint8_t servoID, int woffs, bool flipped) {
  servo[servoNumber].id = servoID;
  servo[servoNumber].woffs = woffs;
  servo[servoNumber].flipped = flipped;
}




//======================================
//====================================== Init leg (set the lenght of coax, femur and tibia in mm)
//======================================

void hexapodSCS::initLegSeize(float _coax, float _femur, float _tibia ) {
  coax = _coax;
  femur = _femur;
  tibia = _tibia;
}

//======================================
//====================================== init leg range (set the min and max degrees for each joint)
//======================================

void hexapodSCS::initMotorAngleRange(int _mincoaxw, int _maxcoaxw, int _minfemurw, int _maxfemurw, int _mintibiaw, int _maxtibiaw) {
  mincoaxw = _mincoaxw;
  maxcoaxw = _maxcoaxw;
  minfemurw = _minfemurw;
  maxfemurw = _maxfemurw;
  mintibiaw = _mintibiaw;
  maxtibiaw = _maxtibiaw;
}

//======================================
//====================================== Init koordinate (set the start coordinates and set offsets, shift from the center of body)
//======================================

void hexapodSCS::initStartCoordinates(uint8_t legNumber, float x, float y, float z, float xoffs, float yoffs, float zoffs) {
  leg[legNumber].x = x;
  leg[legNumber].y = y;
  leg[legNumber].z = z;
  leg[legNumber].yaw = 0;
  leg[legNumber].pitch = 0;
  leg[legNumber].roll = 0;
  leg[legNumber].xHome = x;
  leg[legNumber].yHome = y;
  leg[legNumber].zHome = z;
  leg[legNumber].yawHome = 0;
  leg[legNumber].xCurrent = x;
  leg[legNumber].yCurrent = y;
  leg[legNumber].zCurrent = z;
  leg[legNumber].yawCurrent = 0;
  leg[legNumber].pitchCurrent = 0;
  leg[legNumber].rollCurrent = 0;
  leg[legNumber].xstep = 0;
  leg[legNumber].ystep = 0;
  leg[legNumber].zstep = 0;
  leg[legNumber].yawstep = 0;
  leg[legNumber].pitchstep = 0;
  leg[legNumber].rollstep = 0;

  leg[legNumber].xoffs = xoffs;
  leg[legNumber].yoffs = yoffs;
  leg[legNumber].zoffs = zoffs;

  #ifdef DEBUGHEXA_ANGLE
  Serial.print("Koordinaten Bein:");
  Serial.println(legNumber);
  Serial.print(" x:");
  Serial.print(leg[legNumber].x);
  Serial.print(" y:");
  Serial.print(leg[legNumber].y);
  Serial.print(" z:");
  Serial.println(leg[legNumber].z);
  #endif
}

//======================================
//====================================== Init koordinate move
//======================================

int hexapodSCS::initStartCoordinatesMove(int _servo_time){
  int  beginServoTime = servo_time, beginTimeWait = timeWait;
  servo_time = _servo_time;
  timeWait = _servo_time -1;
  for (int legNumber = 0 ; legNumber < 6 ; legNumber++) {
    calculateW(legNumber);
    if (writeServo(legNumber) == 0) {
      return 0;
    }
  }
  SCservo.EnableTorque(0xfe, 1);
  delay(50);
  #ifdef SERVO_REGWRITE
  SCservo.RegWriteAction(0xfe);
  #endif

  servo_time = beginServoTime;
  timeWait = beginTimeWait;
  return 1;
}

//======================================
//====================================== calibration
//======================================

void hexapodSCS::calibration() {
  for (int i=0; i<=17; i++){
    SCservo.WritePos(i+1,angleMap(90 + (servo[i].flipped ? -servo[i].woffs : servo[i].woffs)) ,1000);
  }
  //leg[legNumber].coaxw += (servo[coaxServoNumber].flipped ? -servo[coaxServoNumber].woffs : servo[coaxServoNumber].woffs);
}

//======================================
//====================================== set new homepoint from current x, y, z coordinates for all legs
//======================================

void hexapodSCS::setNewHomepoint(){
  for (int legNumber =0; legNumber < 6; legNumber++){
    leg[legNumber].xHome = leg[legNumber].x;
    leg[legNumber].yHome = leg[legNumber].y;
    leg[legNumber].zHome = leg[legNumber].z;
    leg[legNumber].yawHome = leg[legNumber].yaw;
    leg[legNumber].pitchHome = leg[legNumber].pitch;
    leg[legNumber].rollHome = leg[legNumber].roll;

    #ifdef DEBUGHEXA_HOMEPOINT
    Serial.print("Bein: ");
    Serial.print(legNumber);
    Serial.print(" zStep: ");
    Serial.print(leg[legNumber].zstep);
    Serial.print(" zHome: ");
    Serial.print(leg[legNumber].zHome);
    Serial.print(" zCurrent: ");
    Serial.println(leg[legNumber].zCurrent);
    #endif
  }

}

//======================================
//====================================== go to new homepoint
//======================================

bool hexapodSCS::gotoHomepoint() {

  for (int legNumber = 0 ; legNumber < 6 ; legNumber++){

    if (leg[legNumber].x != leg[legNumber].xHome || leg[legNumber].y != leg[legNumber].yHome || leg[legNumber].z != leg[legNumber].zHome || leg[legNumber].yaw != leg[legNumber].yawHome ){

      for (int legNumber = 0 ; legNumber < 6 ; legNumber++){

        #ifdef DEBUGHEXA_HOMEPOINT
        Serial.print("Bein: ");
        Serial.print(legNumber);
        Serial.print(" xHome: ");
        Serial.print(leg[legNumber].zHome);
        Serial.print(" yHome: ");
        Serial.print(leg[legNumber].yHome);
        Serial.print(" zHome: ");
        Serial.print(leg[legNumber].zHome);
        Serial.print(" yawHome: ");
        Serial.println(leg[legNumber].zHome);
        #endif



        leg[legNumber].xCurrent = leg[legNumber].x;
        leg[legNumber].yCurrent = leg[legNumber].y;
        leg[legNumber].zCurrent = leg[legNumber].z;
        leg[legNumber].yawCurrent = leg[legNumber].yaw;
        leg[legNumber].pitchCurrent = leg[legNumber].pitch;
        leg[legNumber].rollCurrent = leg[legNumber].roll;

        leg[legNumber].xstep = leg[legNumber].xHome - leg[legNumber].xCurrent;
        leg[legNumber].ystep = leg[legNumber].yHome - leg[legNumber].yCurrent;
        leg[legNumber].zstep = leg[legNumber].zHome - leg[legNumber].zCurrent;
        leg[legNumber].yawstep = leg[legNumber].yawHome - leg[legNumber].yawCurrent;
        leg[legNumber].pitchstep = leg[legNumber].pitchHome - leg[legNumber].pitchCurrent;
        leg[legNumber].rollstep = leg[legNumber].rollHome - leg[legNumber].rollCurrent;


      }

      int beginResolution = resolution, beginServoTime = servo_time, beginTimeWait = timeWait;
      resolution = 10; servo_time = 50; timeWait = 49;
      moveLegsAcclerate();
      resolution =beginResolution; servo_time = beginServoTime; timeWait = beginTimeWait;

    }
  }

  return 1;
}


//======================================
//====================================== set resolution (resolution for calculating how many steps)
//======================================

void hexapodSCS::setMoveTime(int _resolution, int _servo_time) {
  _servo_time = _servo_time / _resolution;
  resolution = _resolution;
  servo_time = _servo_time+40;
  timeWait = _servo_time;
  if (timeWait < 10){ timeWait = 10;}
}

//======================================
//====================================== set speed (speed for the servos)
//======================================




//======================================
//====================================== calculate angle
//======================================

void hexapodSCS::calculateW(uint8_t legNumber) {

  // Bodi IK yaw mit einberechnen. Drehversatz der x und y Koordinaten vom Körpermittelpunkt aus. Körpermittelpunkt wird mit den Offsets zurück gerechnet.
  // Yaw Winkel für die Linke seite invertieren


  float yawDeg = leg[legNumber].yaw / 180 * Pi;
  float pitchDeg = leg[legNumber].pitch / 180 * Pi;
  float rollDeg = leg[legNumber].roll / 180 * Pi;
  float xMiddle = leg[legNumber].x + leg[legNumber].xoffs;
  float yMiddle = leg[legNumber].y + leg[legNumber].yoffs;
  float zMiddle = leg[legNumber].z + leg[legNumber].zoffs;


  /*
  //Drehung um die Z Achse Yaw
  float _x = (xMiddle * cos(yawDeg) - (yMiddle) * sin(yawDeg) - leg[legNumber].xoffs) ;
  float _y = (xMiddle * sin(yawDeg) + (yMiddle) * cos(yawDeg) - leg[legNumber].yoffs) ;
  float _z = zMiddle - leg[legNumber].zoffs;

  //Drehung um Y Achse Roll
  float _x = (xMiddle * cos(yawDeg)  + zMiddle * sin(yawDeg) - leg[legNumber].xoffs) ;
  float _y = yMiddle - leg[legNumber].yoffs;
  float _z = (-xMiddle * sin(yawDeg) + zMiddle * cos(yawDeg));

  //Drehung um X Achse Pitch
  float _x = xMiddle - leg[legNumber].xoffs;
  float _y = (yMiddle * cos(yawDeg) - zMiddle * sin(yawDeg)) - leg[legNumber].yoffs ;
  float _z = (yMiddle * sin(yawDeg) + zMiddle * cos(yawDeg));

  (cos(yawDeg)*cos(rollDeg)) + (cos(yawDeg)*sin(rollDeg)*sin(pitchDeg)-sin(yawDeg)*cos(pitchDeg)) + (cos(yawDeg)*sin(rollDeg)*cos(pitchDeg)+sin(yawDeg)*sin(pitchDeg))
  (sin(yawDeg)*cos(rollDeg)) + (sin(yawDeg)*sin(rollDeg)*sin(pitchDeg)+cos(yawDeg)*cos(pitchDeg)) + (sin(yawDeg)*sin(rollDeg)*cos(pitchDeg)-cos(yawDeg)*sin(pitchDeg))
  -(sin(rollDeg))            + (cos(rollDeg)*sin(pitchDeg))                                       + (cos(rollDeg)*cos(pitchDeg))
  */


  float _x = (cos(yawDeg)*cos(rollDeg)) * xMiddle + (cos(yawDeg)*sin(rollDeg)*sin(pitchDeg)-sin(yawDeg)*cos(pitchDeg)) * yMiddle + (cos(yawDeg)*sin(rollDeg)*cos(pitchDeg)+sin(yawDeg)*sin(pitchDeg)) * zMiddle - leg[legNumber].xoffs;
  float _y = (sin(yawDeg)*cos(rollDeg)) * xMiddle + (sin(yawDeg)*sin(rollDeg)*sin(pitchDeg)+cos(yawDeg)*cos(pitchDeg)) * yMiddle + (sin(yawDeg)*sin(rollDeg)*cos(pitchDeg)-cos(yawDeg)*sin(pitchDeg)) * zMiddle - leg[legNumber].yoffs;
  float _z = -(sin(rollDeg)) * xMiddle            + (cos(rollDeg)*sin(pitchDeg)) * yMiddle                                       + (cos(rollDeg)*cos(pitchDeg)) * zMiddle                                       - leg[legNumber].zoffs;




  #ifdef DEBUGHEXA_YAW
  Serial.print("Bein mit Winkel: ");
  Serial.print(legNumber);
  Serial.print(" .  x . ");
  Serial.print(_x);
  Serial.print(" .  y . ");
  Serial.print(_y);
  Serial.print(" .  z . ");
  Serial.println(_z);
  #endif

  float L1;
  float L;

  L1 = sqrt(sq(_x) + sq(_y));

  //if it not in Range abort
  if (L1 >= (coax + femur + tibia)) {
    //Serial.println("Koordinate nicht im Rahmen");
    return ;
  }

  L = sqrt(sq(_z) + sq(L1 - coax));

  //Gamma = w_hip
  leg[legNumber].coaxw = atan2(_x, _y) * 180 / Pi;

  //Alpha = w_knee
  leg[legNumber].femurw = acos(_z / L) * 180 / Pi + acos((sq(tibia) - sq(femur) - sq(L)) / (-2 * femur * L)) * 180 / Pi;

  //Beta = w_foot
  leg[legNumber].tibiaw  = acos((sq(L) - sq(tibia) - sq(femur)) / (-2 * tibia * femur)) * 180 / Pi;
}




//======================================
//====================================== write to Servo
//======================================

int hexapodSCS::writeServo(uint8_t legNumber) {
  uint8_t coaxServoNumber   =  (legNumber * 3) + 0;
  uint8_t femurServoNumber  = (legNumber * 3) + 1;
  uint8_t tibiaServoNumber  = (legNumber * 3) + 2;

  //Check if Servo flipped
  if (servo[coaxServoNumber].flipped) leg[legNumber].coaxw = leg[legNumber].coaxw + 180;
  if (servo[femurServoNumber].flipped) leg[legNumber].femurw = 180 - leg[legNumber].femurw;
  if (servo[tibiaServoNumber].flipped) leg[legNumber].tibiaw = 180 - leg[legNumber].tibiaw;

  #ifdef DEBUGHEXA_ANGLE
  Serial.print("Raw   Nummer: ");
  Serial.print(legNumber);
  Serial.print(" .  x . ");
  Serial.print(leg[legNumber].x);
  Serial.print(" .  y . ");
  Serial.print(leg[legNumber].y);
  Serial.print(" .  z . ");
  Serial.print(leg[legNumber].z);
  Serial.print(" .  hip . ");
  Serial.print(leg[legNumber].coaxw);
  Serial.print(" .  knee . ");
  Serial.print(leg[legNumber].femurw);
  Serial.print(" .  foot . ");
  Serial.println(leg[legNumber].tibiaw);
  #endif

  // add offset +/- depends if flipped or not
  leg[legNumber].coaxw += (servo[coaxServoNumber].flipped ? -servo[coaxServoNumber].woffs : servo[coaxServoNumber].woffs);
  leg[legNumber].femurw += (servo[femurServoNumber].flipped ? -servo[femurServoNumber].woffs : servo[femurServoNumber].woffs);
  leg[legNumber].tibiaw += (servo[tibiaServoNumber].flipped ? -servo[tibiaServoNumber].woffs : servo[tibiaServoNumber].woffs);

  #ifdef DEBUGHEXA_ANGLE
  if (mincoaxw >= leg[legNumber].coaxw || leg[legNumber].coaxw >= maxcoaxw) {
    Serial.print("Coax Winkel auserhalb Range");

    return 0;
  }
  if (minfemurw >= leg[legNumber].femurw || leg[legNumber].femurw >= maxfemurw) {
    Serial.print("Femur Winkel auserhalb Range");

    return 0;
  }
  if (mintibiaw >= leg[legNumber].tibiaw || leg[legNumber].tibiaw >= maxtibiaw) {
    Serial.print("Tibia Winkel auserhalb Range");

    return 0;
  }
  #endif

  //SCservo.EnableTorque(1, 1);
  #ifdef SERVO_REGWRITE
  SCservo.RegWritePos(servo[coaxServoNumber].id, angleMap(leg[legNumber].coaxw), servo_time);
  SCservo.RegWritePos(servo[femurServoNumber].id, angleMap(leg[legNumber].femurw), servo_time);
  SCservo.RegWritePos(servo[tibiaServoNumber].id, angleMap(leg[legNumber].tibiaw), servo_time);
  #else
  SCservo.WritePos(servo[coaxServoNumber].id, angleMap(leg[legNumber].coaxw), servo_time);
  SCservo.WritePos(servo[femurServoNumber].id, angleMap(leg[legNumber].femurw), servo_time);
  SCservo.WritePos(servo[tibiaServoNumber].id, angleMap(leg[legNumber].tibiaw), servo_time);

  #endif

  #ifdef DEBUGHEXA_ANGLE
  Serial.print("Beine Nummer: ");
  Serial.print(legNumber);
  Serial.print(" .  x . ");
  Serial.print(leg[legNumber].x);
  Serial.print(" .  y . ");
  Serial.print(leg[legNumber].y);
  Serial.print(" .  z . ");
  Serial.print(leg[legNumber].z);
  Serial.print(" .  hip . ");
  Serial.print(leg[legNumber].coaxw);
  Serial.print(" .  knee . ");
  Serial.print(leg[legNumber].femurw);
  Serial.print(" .  foot . ");
  Serial.println(leg[legNumber].tibiaw);
  #endif

  #ifdef DEBUGHEXA3
  Serial.print("Beine Nummer: ");
  Serial.print(legNumber);
  Serial.print(" .  x . ");
  Serial.print(leg[legNumber].x);
  Serial.print(" .  y . ");
  Serial.print(leg[legNumber].y);
  Serial.print(" .  z . ");
  Serial.println(leg[legNumber].z);

  #endif

  return 1;
}

//======================================
// ===================================== map_angle
//======================================

int hexapodSCS::angleMap(int angle)
{
  int position=511;
  position = map(angle,0,180,100,923);
  if (position < 0)position =0;
  if (position > 1023)position =1023;
  return position;
}



//======================================
//====================================== set new coordinates relative to current position
//======================================

int hexapodSCS::setLegCoordRel(uint8_t legNumber, int x, int y, int z, float yaw, float pitch, float roll) {

  leg[legNumber].xCurrent = leg[legNumber].x;
  leg[legNumber].yCurrent = leg[legNumber].y;
  leg[legNumber].zCurrent = leg[legNumber].z;
  leg[legNumber].yawCurrent = leg[legNumber].yaw;
  leg[legNumber].pitchCurrent = leg[legNumber].pitch;
  leg[legNumber].rollCurrent = leg[legNumber].roll;

  leg[legNumber].xstep = x;
  leg[legNumber].ystep = y;
  leg[legNumber].zstep = z;
  leg[legNumber].yawstep = yaw;
  leg[legNumber].pitchstep = pitch;
  leg[legNumber].rollstep = roll;

  #ifdef DEBUGHEXA_STEP_DIMENSION
  Serial.print("Bein: ");
  Serial.print(legNumber);
  Serial.print(" z: ");
  Serial.print(leg[legNumber].z);
  Serial.print(" zStep: ");
  Serial.print(leg[legNumber].zstep);
  Serial.print(" zHome: ");
  Serial.print(leg[legNumber].zHome);
  Serial.print(" zCurrent: ");
  Serial.println(leg[legNumber].zCurrent);
  #endif

  return 1;
}

//======================================
//====================================== set new Leg coordinates absolut
//======================================

int hexapodSCS::setLegCoordAbs(uint8_t legNumber, int x, int y, int z, float yaw, float pitch, float roll) {

  leg[legNumber].xCurrent = leg[legNumber].x;
  leg[legNumber].yCurrent = leg[legNumber].y;
  leg[legNumber].zCurrent = leg[legNumber].z;
  leg[legNumber].yawCurrent = leg[legNumber].yaw;
  leg[legNumber].pitchCurrent = leg[legNumber].pitch;
  leg[legNumber].rollCurrent = leg[legNumber].roll;

  leg[legNumber].xstep = x - leg[legNumber].xCurrent;
  leg[legNumber].ystep = y - leg[legNumber].yCurrent;
  leg[legNumber].zstep = z - leg[legNumber].zCurrent;
  leg[legNumber].yawstep = yaw - leg[legNumber].yawCurrent;
  leg[legNumber].pitchstep = yaw - leg[legNumber].pitchCurrent;
  leg[legNumber].rollstep = yaw - leg[legNumber].rollCurrent;


  return 1;
}


//======================================
//====================================== set new Leg coordinates relative to all legs
//======================================

int hexapodSCS::setAllLegCoordRel( int x, int y, int z, float yaw, float pitch, float roll){
  for (uint8_t legNumber = 0; legNumber < 6; legNumber++) {
    leg[legNumber].xCurrent = leg[legNumber].x;
    leg[legNumber].yCurrent = leg[legNumber].y;
    leg[legNumber].zCurrent = leg[legNumber].z;
    leg[legNumber].yawCurrent = leg[legNumber].yaw;
    leg[legNumber].pitchCurrent = leg[legNumber].pitch;
    leg[legNumber].rollCurrent = leg[legNumber].roll;


    leg[legNumber].xstep = x;
    leg[legNumber].ystep = y;
    leg[legNumber].zstep = z;
    leg[legNumber].yawstep = yaw;
    leg[legNumber].pitchstep = pitch;
    leg[legNumber].rollstep = roll;
  }
  return 1;
}

//======================================
//====================================== set new Bodyhigh coordinates
//======================================

int hexapodSCS::setBodyHighAbs(int z){
  for (uint8_t legNumber = 0; legNumber < 6; legNumber++) {

    leg[legNumber].zCurrent = leg[legNumber].z;

    leg[legNumber].zstep = z - leg[legNumber].zCurrent;


  }
  return 1;
}

//======================================
//====================================== Move legs without Accleration
//======================================

int hexapodSCS::moveLegs() {


  #ifdef DEBUGHEXA_TIME
  int timeStop;
  #endif

  //Steps Loop Auflösung in N Schritte
  for (int i = 0; i < resolution; i++) {



    #ifdef DEBUGHEXA_STEP
    Serial.print("Step Nummer: ");
    Serial.println(i);
    #endif

    //Leg Schritweise Koordinaten erhöhen für jedes Bein, danach an PWM shield senden
    for (uint8_t legNumber = 0; legNumber < 6; legNumber++) {

      #ifdef DEBUGHEXA_STEP_DIMENSION_2
      Serial.print("Bein: ");
      Serial.print(legNumber);
      Serial.print(" z: ");
      Serial.print(leg[legNumber].z);
      Serial.print(" zStep: ");
      Serial.print(leg[legNumber].zstep);
      Serial.print(" zHome: ");
      Serial.print(leg[legNumber].zHome);
      Serial.print(" zCurrent: ");
      Serial.println(leg[legNumber].zCurrent);
      #endif

      leg[legNumber].x = leg[legNumber].x + (leg[legNumber].xstep/resolution);
      leg[legNumber].y = leg[legNumber].y + (leg[legNumber].ystep/resolution);
      leg[legNumber].z = leg[legNumber].z + (leg[legNumber].zstep/resolution);
      leg[legNumber].yaw = leg[legNumber].yaw + (leg[legNumber].yawstep/resolution);
      leg[legNumber].pitch = leg[legNumber].pitch + (leg[legNumber].pitchstep/resolution);
      leg[legNumber].roll = leg[legNumber].roll + (leg[legNumber].rollstep/resolution);

      calculateW(legNumber);
      if (writeServo(legNumber) == 0) {
        return 0;
      }
    }

    // give the servo's time for moving before next movment command
    while(millis()-timeStart < timeWait){    }

    #ifdef DEBUGHEXA_TIME
    timeStop = millis();
    Serial.println(timeStop-timeStart);
    #endif

    #ifdef SERVO_REGWRITE
    SCservo.RegWriteAction(0xfe);
    #endif

    timeStart = millis();

  }
  return 1;
}


//======================================
//====================================== Move legs with Accleration
//======================================

int hexapodSCS::moveLegsAcclerate() {

  #ifdef DEBUGHEXA_TIME
  int timeStop;
  #endif

  for (int i = 1 ; i < resolution + 1; i++) {

    #ifdef DEBUGHEXA_STEP
    Serial.print("Step Nummer: ");
    Serial.println(i);
    #endif

    for (int legNumber = 0 ; legNumber < 6 ; legNumber++) {

      #ifdef DEBUGHEXA_STEP_DIMENSION_2
      Serial.print("Bein: ");
      Serial.print(legNumber);
      Serial.print(" z: ");
      Serial.print(leg[legNumber].z);
      Serial.print(" zStep: ");
      Serial.print(leg[legNumber].zstep);
      Serial.print(" zHome: ");
      Serial.print(leg[legNumber].zHome);
      Serial.print(" zCurrent: ");
      Serial.println(leg[legNumber].zCurrent);
      #endif

      leg[legNumber].x = leg[legNumber].xCurrent + (leg[legNumber].xstep)  * ((cos((M_PI * (double)i / (double)resolution) + M_PI) + (double)1)/2) ;
      leg[legNumber].y = leg[legNumber].yCurrent + (leg[legNumber].ystep)  * ((cos((M_PI * (double)i / (double)resolution) + M_PI) + (double)1)/2) ;
      leg[legNumber].z = leg[legNumber].zCurrent + (leg[legNumber].zstep)  * ((cos((M_PI * (double)i / (double)resolution) + M_PI) + (double)1)/2) ;
      leg[legNumber].yaw = leg[legNumber].yawCurrent - (leg[legNumber].yawstep)  * ((cos((M_PI * (double)i / (double)resolution) + M_PI) + (double)1)/2) ;
      leg[legNumber].pitch = leg[legNumber].pitchCurrent - (leg[legNumber].pitchstep)  * ((cos((M_PI * (double)i / (double)resolution) + M_PI) + (double)1)/2) ;
      leg[legNumber].roll = leg[legNumber].rollCurrent - (leg[legNumber].rollstep)  * ((cos((M_PI * (double)i / (double)resolution) + M_PI) + (double)1)/2) ;


      #ifdef DEBUGHEXA_YAW
      Serial.print("yaw: ");
      Serial.println(leg[legNumber].yaw);
      #endif
      calculateW(legNumber);
      if (writeServo(legNumber) == 0) {
        return 0;
      }
    }
    // give the servo's time for moving before next movment command
    while(millis()-timeStart < timeWait){    }

    #ifdef DEBUGHEXA_TIME
    timeStop = millis();
    Serial.println(timeStop-timeStart);
    #endif
    #ifdef SERVO_REGWRITE
    SCservo.RegWriteAction(0xfe);
    #endif
    timeStart = millis();

  }

  for (int legNumber = 0 ; legNumber < 6 ; legNumber++) {
    leg[legNumber].xCurrent = leg[legNumber].x;
    leg[legNumber].yCurrent = leg[legNumber].y;
    leg[legNumber].zCurrent = leg[legNumber].z;
    leg[legNumber].yawCurrent = leg[legNumber].yaw;
    leg[legNumber].pitchCurrent = leg[legNumber].pitch;
    leg[legNumber].rollCurrent = leg[legNumber].roll;
  }


  return 1;
}

//======================================
//====================================== Walk
//======================================

bool hexapodSCS::walk(int repeat , float amplitudeX, float amplitudeY, float amplitudeZ, float yaw ) {

  // depens on the mechanical flexibility from body. (0.1 for PETG)
  float damping = -0.1;



  bool doFlag = false;
  for (int legNumber = 0 ; legNumber < 6 ; legNumber++){
    leg[legNumber].zstep =0;
    if(tripod_case[legNumber] == 1){
      if (leg[legNumber].z != (leg[legNumber].zHome - amplitudeZ)){
        leg[legNumber].zstep =  amplitudeZ*-1;
        doFlag = true;
      }
    }
  }
  if (doFlag == true){

    int beginResolution = resolution, beginServoTime = servo_time, beginTimeWait = timeWait;
    resolution = 10; servo_time = 50; timeWait = 49;
    moveLegsAcclerate();
    resolution =beginResolution; servo_time = beginServoTime; timeWait = beginTimeWait;
  }




  #ifdef DEBUGHEXA_TIME
  int timeStop;
  #endif

  //for (int ii = 0 ; ii < (repeat * 2); ii++) {


  for (int i = 1 ; i < resolution + 1; i++) {
    #ifdef DEBUGHEXA_STEP
    Serial.print("Step Nummer: ");
    Serial.println(i);
    #endif
    for (int legNumber = 0 ; legNumber < 6 ; legNumber++) {
      if(tripod_case[legNumber] == 1) {
        leg[legNumber].y = leg[legNumber].yHome + amplitudeY * sin(M_PI * ((float)i / (float)resolution) * (float)2);
        leg[legNumber].x = leg[legNumber].xHome + amplitudeX * sin(M_PI* ((float)i / (float)resolution) * (float)2);
        leg[legNumber].z = leg[legNumber].zHome - amplitudeZ * max((float)cos(M_PI*((float)i/(float)resolution) * (float)2),damping);
        leg[legNumber].yaw = leg[legNumber].yawHome - yaw * sin(M_PI * ((float)i / (float)resolution) * (float)2);

        #ifdef DEBUGHEXA_TRIG
        Serial.print("Offsets Bein:");
        Serial.println(legNumber);
        Serial.print(" x:");
        Serial.print(leg[legNumber].x);
        Serial.print(" y:");
        Serial.print(leg[legNumber].y);
        Serial.print(" z:");
        Serial.println(leg[legNumber].z);
        #endif
        //if (i >= resolution) tripod_case[legNumber] = 2;
      }
      if(tripod_case[legNumber] == 2) {
        leg[legNumber].y = leg[legNumber].yHome + amplitudeY * sin(M_PI * (((float)i / (float)resolution) + ((float)2 / (float)4)) * (float)2);
        leg[legNumber].x = leg[legNumber].xHome + amplitudeX * sin(M_PI * (((float)i / (float)resolution) + ((float)2 / (float)4)) * (float)2);
        leg[legNumber].z = leg[legNumber].zHome - amplitudeZ * max((float)cos(M_PI * (((float)i/(float)resolution)+((float)2/(float)4)) * (float)2),damping);
        leg[legNumber].yaw = leg[legNumber].yawHome - yaw * sin(M_PI * (((float)i / (float)resolution) + ((float)2 / (float)4)) * (float)2);
        //leg[legNumber].yaw = leg[legNumber].yawHome + yaw * sin(M_PI * ((float)i / (float)resolution) * (float)2); //+
        //if (i >= resolution) tripod_case[legNumber] = 1;
      }
      #ifdef DEBUGHEXA_YAW
      Serial.print("yaw: ");
      Serial.println(leg[legNumber].yaw);
      #endif
      calculateW(legNumber);
      if (writeServo(legNumber) == 0) {
        return 0;
      }
    }
    // give the servo's time for moving before next movment command

    #ifdef DEBUGHEXA_TIME
    timeStop = millis();
    Serial.print(timeWait);
    Serial.print("  :");
    Serial.println(timeStop-timeStart);
    #endif


    //while (SCservo.ReadMove(0xfe)==1){}
    while(millis()-timeStart < timeWait){    }
    timeStart = millis();
    #ifdef SERVO_REGWRITE
    SCservo.RegWriteAction(0xfe);
    #endif

    //timeStart = millis();
  }

  return 1;
}
