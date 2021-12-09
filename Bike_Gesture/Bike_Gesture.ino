#include <Arduino_APDS9960.h>
void setup() {
  Serial.begin(115200);
  while (!Serial);
  pinMode(LED_BUILTIN, OUTPUT);
  if (!APDS.begin()) {
    Serial.println("Error initializing APDS9960 sensor!");
  }
  Serial.println("Detecting gestures ...");
}
void loop() {
  // put your main code here, to run repeatedly:
if (APDS.gestureAvailable()) {
    // a gesture was detected, read and print to serial monitor
    int gesture = APDS.readGesture();
    switch (gesture) {
      case GESTURE_UP:
        Serial.println("Detected UP gesture");
        digitalWrite(LED_BUILTIN, HIGH);
        break;
      case GESTURE_DOWN:
        Serial.println("Detected DOWN gesture");
        digitalWrite(LED_BUILTIN, LOW);
        break;
      case GESTURE_LEFT:
        Serial.println("Detected LEFT gesture");
        digitalWrite(LED_BUILTIN, LOW);
        break;
      case GESTURE_RIGHT:
        Serial.println("Detected RIGHT gesture");
        digitalWrite(LED_BUILTIN, HIGH);
        break;
      default:
        // ignore
        break;
    }
  }
}
