/*

Coordinates Toe

-z
^     -y
|    /
|  /
+x <--- 0 ---> -x
/ |
/   |
+y    +z

*/




#ifndef hexapodSCS_h
#define hexapodSCS_h

#include <SCServo.h>
#define RXD2 16
#define TXD2 17





class hexapodSCS {
public:
  hexapodSCS() ;

  SCSCL SCservo;
  void begin();
  void attachServo( uint8_t servoNumber, uint8_t servoID, int woffs, bool flipped);                                     // attach servos => number, servo pin, angle offset vor calibrating, flipped turned 180 degrees
  void initLegSeize(float coax, float femur, float tibia);                                                              // set the lenght of coax, femur and tibia in mm
  void initMotorAngleRange(int _mincoaxw, int _maxcoaxw, int _minfemurw, int _maxfemurw, int _mintibiaw, int _maxtibiaw);// set max and min angle range for each joint 0 - 180
  void initStartCoordinates(uint8_t legNumber, float x, float y, float z, float xoffs, float yoffs, float zoffs);       // start coordinates and offsets (shift from the center of body)
  void calibration();                                                                                                   // set all servo to 90 degrees for calibration
  void setNewHomepoint();                                                                                               //set new homepoint from current x, y, z coordinates for all legs
  void setMoveTime(int resolution, int servo_time);                                                                                   // how many steps for one move (resolution)
  // time for one move in milliSecond for SCS15 0 => 180 min 550ms
  void calculateW(uint8_t legNumber);                                                                                   // calculate angel for servo with inverse kinematic
  int writeServo(uint8_t legNumber);                                                                                    // write angle to servo
  int angleMap(int angle);                                                                                              // calculate the pwm signal
  int setCoordinatesRelative(uint8_t legNumber, int x, int y, int z, float yaw);                                        // set new coordinates relative to current position for one leg with accleration
  int setCoordinatesAbsolut(uint8_t legNumber, int x, int y, int z, float yaw);                                         // set new coordinates absolut

  int moveLegs();                                                                                                      // move all legs to te new coordinates no accleration!
  int moveLegsAcclerate();                                                                                              // move all legs to te new coordinates with accleration


  bool walk(int repeat , float amplitudeX, float amplitudeY, float amplitudeZ, float yaw );                                   // Walk function
  bool gotoHomepoint();


  struct legs {
    float x;
    float y;
    float z;
    float xCurrent;
    float yCurrent;
    float zCurrent;
    float xHome;
    float yHome;
    float zHome;
    float yawHome;
    float coaxw;
    float femurw;
    float tibiaw;
    float yaw;
    float pitch;
    float bend;
    float yawCurrent;
    float pitchCurrent;
    float bendCurrent;
    float xoffs;
    float yoffs;
    float zoffs;
    float xstep;
    float ystep;
    float zstep;
    float yawstep;
    float pitchstep;
    float bendstep;
  };
  struct legs leg[6];

  struct servos {
    uint8_t id;
    int woffs;
    bool flipped;
  };

  struct servos servo[18];

  float coax, femur, tibia ;
  float winkel[3];
  int resolution = 1;
  int servo_time = 1000;
  int tripod_case[6] = {1, 2, 1, 2, 1, 2};

private:
  const float Pi = 3.14159265358979323846;
  int mincoaxw, maxcoaxw, minfemurw, maxfemurw, mintibiaw, maxtibiaw;
  int timeWait = 25;
  int timeStart = 0;



};



#endif
