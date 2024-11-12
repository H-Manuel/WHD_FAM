#include <Arduino.h>

// put function declarations here:
int myFunction(int, int);
bool toggle=0;
int LED_pin=6;

void setup() {
  // put your setup code here, to run once:
  int result = myFunction(2, 3);
  pinMode(LED_pin,OUTPUT);
}

void loop() {
  digitalWrite(LED_pin,toggle);
  toggle= !toggle;
  delay(1000);
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}