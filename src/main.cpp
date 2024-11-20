#include <Arduino.h>
#include <Wire.h>
#include "DFRobot_BloodOxygen_S.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include "AsyncUDP.h"

#define MAX30102_I2C_ADDRESS 0x57
#define SHT85_I2C_ADDRESS 0x44
//#define battery 0x36 // MAX17048G+T10
int heartrate=0;
int mean_heartrate=0;
int SPO2=0;
int mean_temperature=0;
int pressure=0;
int PT100_1_pin=4;
int PT100_2_pin=7;
int FSR_pin=16;

DFRobot_BloodOxygen_S_I2C MAX30102(&Wire, MAX30102_I2C_ADDRESS);

// WiFi-Konfiguration
const char *ssid = "Google8Pro"; // SSID des WLANs
const char *pwd = "gavete86";            // Passwort des WLANs
WiFiUDP udp;
IPAddress serverIp;
const int port = 9001;
AsyncUDP udpInput;

void tryConnectWifi()
{
  WiFi.begin(ssid, pwd);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  udp.begin(port);
  serverIp = WiFi.gatewayIP(); // Senden an Standard-Gateway, kann bei Bedarf angepasst werden
  Serial.println("\nConnected to WiFi!");
}

void setupUDP()
{
  if (udpInput.listen(9002))
  {
    Serial.print("UDP Listening on IP: ");
    Serial.println(WiFi.localIP());
    udpInput.onPacket([](AsyncUDPPacket packet)
                      {
      Serial.print("UDP Packet Received: ");
      Serial.println((const char *)packet.data()); });
  }
}

// Function to send a measurement command to the SHT85
void sendSHT85Command(uint16_t command)
{
  Wire.beginTransmission(SHT85_I2C_ADDRESS);
  Wire.write(command >> 8);   // MSB
  Wire.write(command & 0xFF); // LSB
  Wire.endTransmission();
}

// Function to read data from the SHT85
bool readSHT85Data(uint16_t *temperature, uint16_t *humidity)
{
  Wire.requestFrom(SHT85_I2C_ADDRESS, 6); // Request 6 bytes (T MSB, T LSB, T CRC, RH MSB, RH LSB, RH CRC)
  if (Wire.available() == 6)
  {
    uint8_t tempMSB = Wire.read();
    uint8_t tempLSB = Wire.read();
    uint8_t tempCRC = Wire.read();
    uint8_t humMSB = Wire.read();
    uint8_t humLSB = Wire.read();
    uint8_t humCRC = Wire.read();

    // Combine MSB and LSB into 16-bit values
    *temperature = (tempMSB << 8) | tempLSB;
    *humidity = (humMSB << 8) | humLSB;

    // Note: Add CRC check here if required
    return true;
  }
  return false;
}

void setup()
{
  Serial.begin(115200);
  delay(3000);
  Wire.begin(47, 48); // I²C pins

  // Initialize MAX30102
  while (!MAX30102.begin())
  {
    Serial.println("MAX30102 init fail!");
    delay(1000);
  }
  Serial.println("MAX30102 init success!");
  MAX30102.sensorStartCollect();

  // Initialize SHT85
  Serial.println("SHT85 init success! Ready to measure...");

  // WLAN- und UDP-Setup
  tryConnectWifi();
  setupUDP();

  pinMode(PT100_1_pin,INPUT);
  pinMode(PT100_2_pin,INPUT);
  pinMode(FSR_pin,INPUT);

  Serial.println("Setup complete.");
}

void loop()
{
  if (WiFi.status() != WL_CONNECTED) {
    tryConnectWifi();
  }
  // MAX30102 data
  MAX30102.getHeartbeatSPO2();
  Serial.print("SPO2: ");
  Serial.print(MAX30102._sHeartbeatSPO2.SPO2);
  Serial.println("%");

  Serial.print("Heart rate: ");
  Serial.print(MAX30102._sHeartbeatSPO2.Heartbeat);
  Serial.println(" bpm");

  Serial.print("analog1:");
  Serial.println(analogRead(PT100_1_pin));
  Serial.print("analog2:");
  Serial.println(analogRead(PT100_2_pin));
  Serial.print("analog3:");
  Serial.println(analogRead(FSR_pin));
  // SHT85 data
  sendSHT85Command(0x2400); // Single shot measurement command
  delay(1000);              // Wait for measurement to complete (1ms minimum)

  uint16_t temperature, humidity;
  float tempC;
  float humPercent;
  if (readSHT85Data(&temperature, &humidity))
  {
    tempC = -45.0 + 175.0 * ((float)temperature / 65535.0);
    humPercent = 100.0 * ((float)humidity / 65535.0);

    Serial.print("Temperature: ");
    Serial.print(tempC);
    Serial.println(" °C");

    Serial.print("Humidity: ");
    Serial.print(humPercent);
    Serial.println(" %");
  }
  else
  {
    Serial.println("SHT85 read failed!");
  }

  udp.beginPacket(serverIp, port);
  String message = String(heartrate)+";"+ String(mean_heartrate)+";"+ String(SPO2)+";"+ String(mean_temperature)+";"+ String(humPercent)+";"+ String(pressure);
  udp.println(message);
  udp.endPacket();
  delay(4000); // Delay between measurements
}
