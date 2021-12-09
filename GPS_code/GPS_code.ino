
#include <Arduino.h>
#include "wiring_private.h"
#define GPS_RX_PIN 4
#define GPS_TX_PIN 3
 
UART mygps(digitalPinToPinName(GPS_TX_PIN), digitalPinToPinName(GPS_RX_PIN), NC, NC);
void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
pinMode(GPS_RX_PIN, INPUT);
pinMode(GPS_TX_PIN, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  while (mygps.available()>0){
    byte gpsData = mygps.read();
    Serial.write(gpsData);
  }
}
