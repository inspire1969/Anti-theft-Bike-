
#include <Arduino.h>
#include "wiring_private.h"
#define GSM_RX_PIN 3
#define GSM_TX_PIN 4
 
UART mySerial(digitalPinToPinName(GSM_TX_PIN), digitalPinToPinName(GSM_RX_PIN), NC, NC);
 
 
void setup() {
Serial.begin(9600); // connect serial
pinMode(GSM_RX_PIN, INPUT);
pinMode(GSM_TX_PIN, OUTPUT);
}
void loop() {
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
void SendMessage()
{
  mySerial.println("AT+CMGF=1");    //Sets the GSM Module in Text Mode
  delay(1000);  // Delay of 1000 milli seconds or 1 second
  mySerial.println("AT+CMGS=\"+4915207413711\"\r"); // Replace x with mobile number
  delay(1000);
  mySerial.println("I am SMS from GSM Module");// The SMS text you want to send
  delay(100);
   mySerial.println((char)26);// ASCII code of CTRL+Z
  delay(1000);
}
void RecieveMessage()
{
  mySerial.println("AT+CNMI=2,2,0,0,0"); // AT Command to receive a live SMS
  delay(1000);
 }
