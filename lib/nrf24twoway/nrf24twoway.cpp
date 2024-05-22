
#include "nrf24twoway.h"
#define DEBUG1

nrf24twoway::nrf24twoway() {

}


void nrf24twoway::begin(uint8_t transmiter) {
  radio.begin();
  //radio.setDataRate( RF24_250KBPS );
  radio.setChannel(120);
  radio.setDataRate( RF24_250KBPS );
  radio.setPALevel(RF24_PA_HIGH);
  switch (transmiter) {
    case 1:
    radio.openReadingPipe(1, radioAddress[1]);
    radio.openWritingPipe(radioAddress[0]);
    break;
    case 2:
    radio.openReadingPipe(1, radioAddress[0]);
    radio.openWritingPipe(radioAddress[1]);
    break;
    default:
    Serial.println("Transmiter number invalid");
    break;
  }


  radio.startListening();
}

int nrf24twoway::getData(String *buf)
{
  if ( radio.available() )
  {
    radio.read( &dataReceived, sizeof(dataReceived) );
    *buf = dataReceived;
    memset(dataReceived, 0, sizeof(dataReceived));
    return 1;
  }
  return 0;

}

void nrf24twoway::sendData(String buf2) {
  radio.stopListening();
  char* _buf2 = new char[buf2.length() + 1];
  strcpy(_buf2, buf2.c_str());
  //Serial.print("_buf2: ");
  //Serial.print(buf2);
  //Serial.print(" sizeof _buf2: ");
  //Serial.println(buf2.length());
  //Serial.println(buf2.length() + 1);
  radio.write(_buf2, buf2.length() + 1);
  radio.startListening();
  delete[] _buf2;
}

void nrf24twoway::sendDataChecked(String buf2) {
  radio.stopListening();
  char* _buf2 = new char[buf2.length() + 1];
  strcpy(_buf2, buf2.c_str());
  //Serial.print("_buf2: ");
  //Serial.print(buf2);
  //Serial.print(" sizeof _buf2: ");
  //Serial.println(buf2.length());
  //Serial.println(buf2.length() + 1);
  radio.write(_buf2, buf2.length() + 1);
  radio.startListening();

  int i=0;
  while ( radio.available() == 0 )
  {

    if (i == 20){
      radio.stopListening();
      radio.write(_buf2, buf2.length() + 1);
      radio.startListening();
    }
    delay(1);
    i++;
  }

  radio.read( &dataReceived, sizeof(dataReceived) );
  memset(dataReceived, 0, sizeof(dataReceived));
  delete[] _buf2;
}

void nrf24twoway::sendDataSerial() {
  int i = 0;
  while (Serial.available() > 0 && i <= charsize-1) {
    radio.stopListening();
    dataSend[i] = Serial.read();
    i++;
    delay(5);
  }
  if (i != 0) {
    radio.write(&dataSend, sizeof(dataSend));
    memset(dataSend, 0, sizeof(dataSend));
    radio.startListening();
  }
}
