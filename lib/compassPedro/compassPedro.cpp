#include <Arduino.h>
#include "compassPedro.h"

compassPedro::compassPedro() {

}

void compassPedro::begin(){
  //Compass Init
  compass.begin();
  compass.setRange(HMC5883L_RANGE_1_3GA);
  compass.setMeasurementMode(HMC5883L_CONTINOUS);
  compass.setDataRate(HMC5883L_DATARATE_7_5HZ);
  compass.setSamples(HMC5883L_SAMPLES_8);
  compass.setOffset(0, 0);

}

int compassPedro::getHeadingDegrees(){
  VectorCompass norm = compass.readNormalize();

  // Calculate heading
  heading = atan2(norm.YAxis, norm.XAxis);

  // Set declination angle on your location and fix heading
  // You can find your declination on: http://magnetic-declination.com/
  // (+) Positive or (-) for negative
  // For Bytom / Poland declination angle is 4'26E (positive)
  // Formula: (deg + (min / 60.0)) / (180 / M_PI);
  declinationAngle = (2.0 + (55.0 / 60.0)) / (180 / M_PI);
  heading += declinationAngle;

  // Correct for heading < 0deg and heading > 360deg
  if (heading < 0)
  {
    heading += 2 * PI;
  }

  if (heading > 2 * PI)
  {
    heading -= 2 * PI;
  }

  // Convert to degrees
  headingDegreesRaw = heading * 180/M_PI;
  if (abs((int)headingDegreesRaw - headingDegrees) > 0.9){
    headingDegrees = (int)headingDegreesRaw;
  }



  return headingDegrees;
}
