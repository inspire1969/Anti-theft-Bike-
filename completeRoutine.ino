/*
  Callback LED

  This example creates a BLE peripheral with service that contains a
  characteristic to control an LED. The callback features of the
  library are used.

  The circuit:
  - Arduino MKR WiFi 1010, Arduino Uno WiFi Rev2 board, Arduino Nano 33 IoT,
    Arduino Nano 33 BLE, or Arduino Nano 33 BLE Sense board.

  You can use a generic BLE central app, like LightBlue (iOS and Android) or
  nRF Connect (Android), to interact with the services and characteristics
  created in this sketch.

  This example code is in the public domain.
*/

#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>
#include "wiring_private.h"
#include "TinyGPS++.h"

int State = 0;

BLEService ledService("19B10000-E8F2-537E-4F6C-D104768A1214"); // create service

// create switch characteristic and allow remote device to read and write
BLEByteCharacteristic switchCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);

const int ledPin = LED_BUILTIN; // pin to use for the LED
const int ledPinGreen = 11;
const int ledPinRed = 10;
const int ledPinYellow = 9;
#define BuzzerPin 8
#define GPS_RX_PIN 6
#define GPS_TX_PIN 5

// create the tinygps object globally
TinyGPSPlus gps;

// create a software Global UART connection on Pin GPS_TX_PIN and GPS_RX_PIN
UART mySerial1(digitalPinToPinName(GPS_RX_PIN), digitalPinToPinName(GPS_TX_PIN), NC, NC);
UART mySerial2(digitalPinToPinName(GPS_RX_PIN), digitalPinToPinName(GPS_TX_PIN), NC, NC);



void toggle(int PIN){
  digitalWrite(PIN, !digitalRead(PIN)); 
}

bool checkThreshold(float x, float y, float z){
  if (abs(x) < 0.75 && abs(y) < 0.75 && z < 1.5 && z > 0.5){
    return false; // normal accl. behavior
  }
  else{
    return true; // abnormal accl. behavior -> trigger alarm
  }
}


void GPSInit(){
  mySerial1.begin(9600);
}

void GPSReceive(byte* LAT, byte* LNG){
  if (mySerial1.available() > 0){
    byte gpsData = mySerial1.read();
    gps.encode(gpsData);
    *LAT = gps.location.lat();
    *LNG = gps.location.lng();
  }
}

void GSMInit(){
   mySerial2.begin(9600)
 }
void GSMSendMessage{
if (Serial.available()>0)
   switch(Serial.read())
  {
    case 's':
      SendMessage();
      break;
    case 'r':
      RecieveMessage();
      break;
  }

 if (mySerial.available()>0)
   Serial.write(mySerial.read());
 }
 


bool alarmState = false;


void disableAllLed(){
  digitalWrite(ledPinGreen, LOW);
  digitalWrite(ledPinRed, LOW);
  digitalWrite(ledPinYellow, LOW);
}


void setup() {
  Serial.begin(9600);
  while (!Serial);

  // led init
  pinMode(ledPin, OUTPUT); // use the LED pin as an output
  pinMode(ledPinRed, OUTPUT);
  pinMode(ledPinGreen, OUTPUT);
  pinMode(ledPinYellow, OUTPUT);

  
  // begin BLE initialization
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");

    while (1);
  }

  // set the local name peripheral advertises
  BLE.setLocalName("LEDCallback");
  // set the UUID for the service this peripheral advertises
  BLE.setAdvertisedService(ledService);

  // add the characteristic to the service
  ledService.addCharacteristic(switchCharacteristic);

  // add service
  BLE.addService(ledService);

  // assign event handlers for connected, disconnected to peripheral
  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  // assign event handlers for characteristic
  switchCharacteristic.setEventHandler(BLEWritten, switchCharacteristicWritten);
  // set an initial value for the characteristic
  switchCharacteristic.setValue(0);

  // start advertising
  BLE.advertise();

  Serial.println(("Bluetooth device active, waiting for connections..."));


  //GPS Init
  GPSInit();


  //IMU Init
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
}

void loop() {
  // poll for BLE events
  BLE.poll(1000);
  switch(State){
    case 1: //locked
      float x, y, z;
      disableAllLed();
      noTone(BuzzerPin);
      if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(x, y, z);
      }
      //Serial.print(x); Serial.print(y); Serial.print(z);
      alarmState = checkThreshold(x,y,z);
      if (alarmState){
        State = 5; // Switch State to alarm sequence
        break;
      }
      toggle(ledPinYellow);
      Serial.println("Bike is locked.");
      break;


      
    case 2: //unlocked
      disableAllLed();
      noTone(BuzzerPin);
      // currently idle, add crash detection later
      toggle(ledPinGreen);
      Serial.println("Bike unlocked.");
      break;


      
    case 5: //alarm
      disableAllLed();
      byte Lat, Lng;
      GPSReceive(&Lat, &Lng);
      toggle(ledPinRed);
      tone(BuzzerPin, 1000);
      Serial.println("Suspicious Activity detected!");
      Serial.print("LAT="); Serial.print(Lat); Serial.print(" LNG="); Serial.println(Lng);

      if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(x, y, z);
      }
      Serial.print(x); Serial.print(y); Serial.println(z);
      break;

      
    default:
      State = 1;
      Serial.println("Default Case // Switch to Locked");
      
  }
}

void blePeripheralConnectHandler(BLEDevice central) {
  // central connected event handler
  Serial.print("Connected event, central: ");
  Serial.println(central.address());
  State = 0;
}

void blePeripheralDisconnectHandler(BLEDevice central) {
  // central disconnected event handler
  Serial.print("Disconnected event, central: ");
  Serial.println(central.address());
  State = 0;
}

void switchCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  // central wrote new value to characteristic, update LED
  Serial.print("Characteristic event, written: ");

  State = switchCharacteristic.value();
  Serial.print(State);

}
