#include <Arduino.h>
#include <Wire.h>
#include "DFRobot_BloodOxygen_S.h"

#define I2C_ADDRESS 0x57
DFRobot_BloodOxygen_S_I2C MAX30102(&Wire, I2C_ADDRESS);
int PT100_1_pin=4;
int PT100_2_pin=7;
int Pre_pin=9;

void setup() {
  pinMode(PT100_1_pin, OUTPUT);
  pinMode(PT100_2_pin, OUTPUT);
  //pinMode(Pre_pin, INPUT);


  Serial.begin(115200);
  delay(3000);
  Wire.begin(47, 48);
  while (!MAX30102.begin()) {
    Serial.println("init fail!");
    delay(1000);
  }
  Serial.println("init success!");
  Serial.println("start measuring...");
  MAX30102.sensorStartCollect(); // Start data collection
}

void loop() {
  
  digitalWrite(PT100_1_pin,0);
  digitalWrite(PT100_2_pin,1);
  Serial.println(analogRead(Pre_pin));
  MAX30102.getHeartbeatSPO2();
  
  // Print SPO2 and heart rate
  Serial.print("SPO2 is : ");
  Serial.print(MAX30102._sHeartbeatSPO2.SPO2);
  Serial.println("%");
  
  Serial.print("heart rate is : ");
  Serial.print(MAX30102._sHeartbeatSPO2.Heartbeat);
  Serial.println(" Times/min");
  
  // Wait before polling again
  delay(1000); // The sensor updates data every 4 seconds
}