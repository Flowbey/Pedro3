#include <Arduino.h>
#include <U8x8lib.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "hexapodSCS.h"
#include "nrf24twoway.h"                            // make sure to set the CE and CSN pins in nrf24twoway.h file
#include "StringSplitter.h"
#include "hexapodHead.h"
#include "compassPedro.h"

//#define ENABLE_DEBUG1
#define ENABLE_DEBUG_OLED

// Function declaration
void parseCom();
void idleLegReset();
void autoNavigate();

// Value is for storing the command
int value[8] = {0, 0, 0, 0, 0, 0, 0, 0};

int servoTime = 2000;                 // minimum 25 millis
int resolution = 15;
int idleTime = 1000;
int idleTimer1 = 0;
bool idleTimer1on = false;
String radioData;
String done = "done";
String walkFurther = "walkFurther";

//select autonomus modus
// 0 = off
// 1 = "Line Following
// 2 = "CCC"
// 3 = "follow compass"
int modusPaul = 0;

String reset = "reset";
String badRange = "Bad range";
String noCommand = "No command detected";

// Display Init
U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);

//Init Pixy2
hexapodHead paulHead;

nrf24twoway funk;

compassPedro paulCompass;

class hexapodSCS paul;

enum servos { front_left_coax, front_left_femur, front_left_tibia,
  middle_left_coax, middle_left_femur, middle_left_tibia,
  rear_left_coax, rear_left_femur, rear_left_tibia,
  front_right_coax, front_right_femur, front_right_tibia,
  middle_right_coax, middle_right_femur, middle_right_tibia,
  rear_right_coax, rear_right_femur, rear_right_tibia
};
enum legs { front_left,
  middle_left,
  rear_left,
  front_right,
  middle_right,
  rear_right
};


void setup() {

  Serial.begin(115200);
  delay(100);
  Serial.println("Start");
  funk.begin(2);                 // 1 for sender 2 for receiver
  delay(100);

  //Display Init
  u8x8.begin();
  u8x8.setPowerSave(0);
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  u8x8.setFlipMode(1);
  delay(100);

  paul.begin();

  paulHead.begin(19,20);

  paulCompass.begin();

  delay(200);

  // SoftwareID, HardwareID, Anglemiddlepoint Offset, Flipped?
  // uint8_t tibiaDesignW =11.3; // Leg Typ A
  uint8_t tibiaDesignW =0; // Leg Typ B
  uint8_t coaxDesignW = 35;

  //coax, femur and tibia length
  //paul.initLegSeize(54, 74, 127.4); // Leg Typ A
  paul.initLegSeize(54, 74, 124); // Leg Typ B

  //max and min Range of Angle
  paul.initMotorAngleRange(1, 179, 1, 179, 1, 179);

  paul.attachServo(front_left_coax,     1, 3 + coaxDesignW, 0);
  paul.attachServo(front_left_femur,    2, -5, 0);
  paul.attachServo(front_left_tibia,    3, 13.5 - tibiaDesignW, 1);
  paul.attachServo(middle_left_coax,    4, 1, 0);
  paul.attachServo(middle_left_femur,   5, 3, 0);
  paul.attachServo(middle_left_tibia,   6, 2 - tibiaDesignW, 1);
  paul.attachServo(rear_left_coax,      7, -9 - coaxDesignW, 0);
  paul.attachServo(rear_left_femur,     8, -2, 0);
  paul.attachServo(rear_left_tibia,     9, 1 - tibiaDesignW, 1);
  paul.attachServo(front_right_coax,    10, 2 + coaxDesignW, 1);
  paul.attachServo(front_right_femur,   11, 1, 0);
  paul.attachServo(front_right_tibia,   12, 0 - tibiaDesignW, 1);
  paul.attachServo(middle_right_coax,   13, -7, 1);
  paul.attachServo(middle_right_femur,  14, 4, 0);
  paul.attachServo(middle_right_tibia,  15, -3 - tibiaDesignW , 1);
  paul.attachServo(rear_right_coax,     16, 6 - coaxDesignW, 1);
  paul.attachServo(rear_right_femur,    17, -4, 0);
  paul.attachServo(rear_right_tibia,    18, 2 - tibiaDesignW, 1);

  // X , Y , Z , Xoffset , Yoffset , Zoffset (Offsets from body middlepoint absolut)
  paul.initStartCoordinates(front_left,    100, 85, 30, 66.492, 86.345, 0);
  paul.initStartCoordinates(middle_left,   140, 0, 30, 81.5, 0, 0);
  paul.initStartCoordinates(rear_left,     100, -85, 30, 66.492, -86.345, 0);
  paul.initStartCoordinates(front_right,   -100, 85, 30, -66.492, 86.345, 0);
  paul.initStartCoordinates(middle_right,  -140, 0, 30, -81.5, 0, 0);
  paul.initStartCoordinates(rear_right,    -100, -85, 30, -66.492, -86.345, 0);

  /*
  paul.initStartCoordinates(front_left,    120, 65, 30, 66.492, 86.345, 0);
  paul.initStartCoordinates(middle_left,   140, 0, 30, 81.5, 0, 0);
  paul.initStartCoordinates(rear_left,     120, -65, 30, 66.492, -86.345, 0);
  paul.initStartCoordinates(front_right,   -120, 65, 30, -66.492, 86.345, 0);
  paul.initStartCoordinates(middle_right,  -140, 0, 30, -81.5, 0, 0);
  paul.initStartCoordinates(rear_right,    -120, -65, 30, -66.492, -86.345, 0);
  */

  //how many steps and total move time
  paul.setMoveTime(resolution, servoTime);



  //Write initStartCoordinates
  paul.initStartCoordinatesMove(2000);


  u8x8.setCursor(0, 0);
  u8x8.print("Ready");


  Serial.println("Ready");


  /*
  FOR TEST
  modusPaul = 1;
  #ifdef ENABLE_DEBUG_OLED
  u8x8.clearLine(0);
  u8x8.setCursor(0, 0);
  u8x8.print("Linie folgen");
  #endif
  paulHead.beginFollowLine();
  //For Test
  */
  paulHead.headMoveAbsolut(90,40);
}



void loop() {
  parseCom();
  switch (modusPaul){
    case 1:
    paulHead.scanLineSingle();
    paul.walk(1, 0 , paulHead.lineFollowWide, 20, paulHead.lineFollowDeviation);
    break;

    case 2:
    paulHead.scanCCC();
    paulHead.FollowCCC();
    break;

    default:
    break;
  }
  idleLegReset();

  u8x8.setCursor(0, 5);
  u8x8.print(paulCompass.getHeadingDegrees());
  //paulHead.getDistance();
  //paulHead.test_leds(1000);
  //delay(50);

}

void parseCom() {
  if (funk.getData(&radioData)) {
    StringSplitter *splitter = new StringSplitter(radioData, ',', 8);  // new StringSplitter(string_to_split, delimiter, limit)
    int itemCount = splitter->getItemCount();
    #ifdef ENABLE_DEBUG1
    Serial.println("Test String: " + radioData);
    Serial.println("Item count: " + String(itemCount));
    #endif
    for (int i = 0; i < itemCount; i++) {
      String item = splitter->getItemAtIndex(i);
      value[i] = item.toInt();
    }
    delete splitter;

    #ifdef ENABLE_DEBUG1
    for (int i = 0; i < sizeof(value) / sizeof(value[0]); i++) {
      Serial.println(value[i]);
    }
    #endif

    switch (value[0]) {
      case 1:
      modusPaul = 0;
      funk.sendData(done);
      break;

      case 2:
      modusPaul = 0;
      paul.calibration();
      funk.sendData(done);
      break;

      case 3:
      modusPaul = 0;
      #ifdef ENABLE_DEBUG_OLED
      u8x8.clearLine(0);
      u8x8.setCursor(0, 0);
      u8x8.print("Aufstehen");
      #endif

      paul.setLegCoordRel(front_left,   0,    0,    40, 0,  0,  0);
      paul.setLegCoordRel(middle_left,  0,    0,    40, 0,  0,  0);
      paul.setLegCoordRel(rear_left,    0,    0,    40, 0,  0,  0);
      paul.setLegCoordRel(front_right,  0,    0,    40, 0,  0,  0);
      paul.setLegCoordRel(middle_right, 0,    0,    40, 0,  0,  0);
      paul.setLegCoordRel(rear_right,   0,    0,    40, 0,  0,  0);
      paul.moveLegsAcclerate();
      //paul.moveLegs();
      paul.setNewHomepoint();
      funk.sendData(done);
      break;

      case 4:
      modusPaul = 0;
      #ifdef ENABLE_DEBUG_OLED
      u8x8.clearLine(0);
      u8x8.setCursor(0, 0);
      u8x8.print("Setzen");
      #endif

      paul.setLegCoordRel(front_left,   0,    0,   -40, 0,  0,  0);
      paul.setLegCoordRel(middle_left,  0,    0,   -40, 0,  0,  0);
      paul.setLegCoordRel(rear_left,    0,    0,   -40, 0,  0,  0);
      paul.setLegCoordRel(front_right,  0,    0,   -40, 0,  0,  0);
      paul.setLegCoordRel(middle_right, 0,    0,   -40, 0,  0,  0);
      paul.setLegCoordRel(rear_right,   0,    0,   -40, 0,  0,  0);
      paul.moveLegsAcclerate();
      paul.setNewHomepoint();
      funk.sendData(done);
      break;

      case 5:
      modusPaul = 0;
      #ifdef ENABLE_DEBUG_OLED
      u8x8.clearLine(0);
      u8x8.setCursor(0, 0);
      u8x8.print("Laufen");
      #endif
      paul.walk(value[1], value[2], value[3], value[4], value[5]);
      idleTimer1 = millis();
      idleTimer1on = true;
      //funk.sendDataChecked(done);
      funk.sendData(done);
      break;

      case 6:
      break;

      case 7:
      modusPaul = 0;
      paul.setLegCoordRel(front_left,   0,    0,   0, value[5],  0,  0);
      paul.setLegCoordRel(middle_left,  0,    0,   0, value[5],  0,  0);
      paul.setLegCoordRel(rear_left,    0,    0,   0, value[5],  0,  0);
      paul.setLegCoordRel(front_right,  0,    0,   0, value[5],  0,  0);
      paul.setLegCoordRel(middle_right, 0,    0,   0, value[5],  0,  0);
      paul.setLegCoordRel(rear_right,   0,    0,   0, value[5],  0,  0);
      paul.moveLegsAcclerate();
      funk.sendData(done);
      break;

      case 8:
      modusPaul = 0;
      paulHead.headMoveRelative(value[2], value[3]);
      //delay(60);
      //funk.sendDataChecked(done);
      break;

      case 9:
      modusPaul = 0;                                // Speed Adjust
      paul.setMoveTime(resolution, value[1]);
      funk.sendData(done);
      break;

      case 10:
      modusPaul = 0;
      paul.gotoHomepoint();                        // High Adjust
      //paul.setAllLegCoordRel(0,    0,   value[1], 0);
      paul.setBodyHighAbs( value[1]);
      paul.moveLegsAcclerate();
      paul.setNewHomepoint();
      funk.sendData(done);
      break;

      case 20:
      modusPaul = 0;
      funk.sendData(done);
      break;

      case 30:
      modusPaul = 1;
      #ifdef ENABLE_DEBUG_OLED
      u8x8.clearLine(0);
      u8x8.setCursor(0, 0);
      u8x8.print("Linie folgen");
      #endif
      paulHead.headMoveAbsolut(90,140);
      break;

      case 40:
      modusPaul = 2;
      #ifdef ENABLE_DEBUG_OLED
      u8x8.clearLine(0);
      u8x8.setCursor(0, 0);
      u8x8.print("Color folgen");
      #endif

      break;

      case 50: //For Testing
      #ifdef ENABLE_DEBUG_OLED
      u8x8.clearLine(0);
      u8x8.setCursor(0, 0);
      u8x8.print("Command 50");
      #endif
      paul.setAllLegCoordRel(   value[2],    value[3],   value[4],   value[5],  value[6],  value[7]);

      paul.moveLegsAcclerate();
      funk.sendData(done);
      break;

      case 51: //For Testing
      #ifdef ENABLE_DEBUG_OLED
      u8x8.clearLine(0);
      u8x8.setCursor(0, 0);
      u8x8.print("Command 51");
      #endif
      paul.setAllLegCoordRel(   value[2],    value[3],   value[4],   value[5],  value[6],  value[7]);

      paul.moveLegsAcclerate();
      funk.sendData(done);
      break;


      default:
      modusPaul = 0;
      funk.sendData(noCommand);
      break;
    }
  }
}

void idleLegReset() {
  if (idleTimer1on == true){
    if (idleTimer1 + idleTime < millis()){
      paul.gotoHomepoint();
      idleTimer1on = false;
    }
  }
}

void autoNavigate(){

}
