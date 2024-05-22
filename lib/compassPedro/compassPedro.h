#ifndef _compassPedro_h
#define _compassPedro_h

#include <Arduino.h>
#include "HMC5883L.h"


class compassPedro {
public:
  compassPedro();
  void begin();
  int getHeadingDegrees();



private:
  HMC5883L compass;
  int headingDegrees;
  float headingDegreesRaw;
  float declinationAngle;
  float heading;

};



#endif
