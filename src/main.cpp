#include <Arduino.h>
#include <Wire.h>
#include "DFRobot_BloodOxygen_S.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include "AsyncUDP.h"
#include "Adafruit_MAX1704X.h"
#include <string>

#define MAX30102_I2C_ADDRESS 0x57
#define SHT85_I2C_ADDRESS 0x44
#define BATTERY_I2C_ADDRESS 0x36 // MAX17048G+T10
// int heartrate = 0; //
int mean_heartrate = 0;
String heart_string;
int arr_heartrate[5] = {};
int arr_counter = 0;
// int SPO2 = 0; //
String SPO2_string;
// int mean_temperature = 0;
int pressure = 0;
int start_pressure = 0;
String pressure_sting;
int PT100_1_pin = 4;
int PT100_2_pin = 7;
int FSR_pin = 16;
int prev_heartrate = 0;
int prev_SPO2 = 0;
float tempC = 0;
String temp_string;
float humPercent = 0;
String hum_string;
int wifi_counter = 0;

DFRobot_BloodOxygen_S_I2C MAX30102(&Wire, MAX30102_I2C_ADDRESS);
Adafruit_MAX17048 maxlipo;

// WiFi-Konfiguration
const char *ssid = "Google8Pro"; // SSID des WLANs
const char *pwd = "gavete86";    // Passwort des WLANs
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
    /*
    wifi_counter+=1;
    if (wifi_counter > 60) {
      Serial.println("\nFailed to connect to WiFi after 30 seconds.");
      return; // Verlasse die Funktion, wenn keine Verbindung hergestellt wurde
    }
    */
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

void setup_battery(){
  maxlipo.begin(&Wire);
  while (!maxlipo.begin()) {
    Serial.println(F("Couldnt find Adafruit MAX17048?\nMake sure a battery is plugged in!"));
    delay(2000);
  }
  Serial.print(F("Found MAX17048"));
  Serial.print(F(" with Chip ID: 0x")); 
  Serial.println(maxlipo.getChipID(), HEX);
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

int heartreat_read()
{
  if (MAX30102._sHeartbeatSPO2.Heartbeat > 0)
  {
    prev_heartrate = MAX30102._sHeartbeatSPO2.Heartbeat-15;
    return MAX30102._sHeartbeatSPO2.Heartbeat-15;
  }
  else
  {
    return prev_heartrate;
  }
}

int SPO2_read()
{
  if (MAX30102._sHeartbeatSPO2.SPO2 > 0)
  {
    prev_SPO2 = MAX30102._sHeartbeatSPO2.SPO2;
    return MAX30102._sHeartbeatSPO2.SPO2;
  }
  else
  {
    return prev_SPO2;
  }
}

int pressure_read()
{
  pressure = map(analogRead(FSR_pin), 4095, 2000, 0, 100);
  return pressure;
}

int mean_heartrate_read()
{
  // Verschiebe alle Werte um eins nach hinten
  for (int i = 4; i > 0; i--)
  {
    arr_heartrate[i] = arr_heartrate[i - 1];
  }

  // Füge den neuen Wert an der ersten Stelle ein
  arr_heartrate[0] = heartreat_read();

  // Berechne den Durchschnitt
  mean_heartrate = 0;
  for (int i = 0; i < (sizeof(arr_heartrate) / sizeof(arr_heartrate[0])); i++)
  {
    mean_heartrate += arr_heartrate[i];
  }

  return mean_heartrate / 5;
}

void print()
{
  Serial.print("SPO2: ");
  Serial.print(SPO2_read());
  Serial.println("%");

  Serial.print("Heartrate: ");
  Serial.print(heartreat_read());
  Serial.println(" bpm");
  /*
  for (int i = 0; i < (sizeof(arr_heartrate) / sizeof(arr_heartrate[0])); i++)
  {
    Serial.println(arr_heartrate[i]);
  }
  */
  Serial.print("Heartrate mean: ");
  Serial.print(mean_heartrate_read());
  Serial.println(" bpm");

  Serial.print("analog1:");
  Serial.println(analogRead(PT100_1_pin));
  Serial.print("analog2:");
  Serial.println(analogRead(PT100_2_pin));
  Serial.print("analog3 maped:");
  Serial.println(pressure_read());

  Serial.print("Temperature: ");
  Serial.print(tempC);
  Serial.println(" °C");

  Serial.print("Humidity: ");
  Serial.print(humPercent);
  Serial.println(" %");

  /*Serial.print("Battery: ");
  Serial.print(maxlipo.cellPercent(), 1);
  Serial.println(" %");
  */
}

void strings_create()
{
  if (pressure_read() - 20 > start_pressure)
  {
    pressure_sting = "high";
  }
  else if (pressure_read() + 20 < start_pressure)
  {
    pressure_sting = "low";
  }
  else
  {
    pressure_sting = "stable";
  }

  if (heartreat_read() - 3 > 190)
  {
    heart_string = "high";
  }
  else if (pressure_read() + 3 < 30)
  {
    heart_string = "low";
  }
  else
  {
    heart_string = "stable";
  }

  if (SPO2_read() > 100)
  {
    SPO2_string = "high";
  }
  else if (SPO2_read() < 92)
  {
    SPO2_string = "low";
  }
  else
  {
    SPO2_string = "stable";
  }

  if (tempC > 38)
  {
    temp_string = "high";
  }
  else if (pressure_read() < 92)
  {
    temp_string = "low";
  }
  else
  {
    temp_string = "stable";
  }

  if (humPercent > 60)
  {
    hum_string = "to wet";
  }
  else if (pressure_read() < 40)
  {
    hum_string = "too dry";
  }
  else
  {
    hum_string = "stable";
  }
}

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    delay(10);
  Wire.begin(47, 48); // I²C pins

  // Initialize MAX30102
  while (!MAX30102.begin())
  {
    Serial.println("MAX30102 init fail!");
    delay(1000);
  }

  MAX30102.sensorStartCollect();

  // Initialize SHT85
  Serial.println("SHT85 init success! Ready to measure...");

  // WLAN- und UDP-Setup
  tryConnectWifi();
  setupUDP();
  //setup_battery();

  pinMode(PT100_1_pin, INPUT);
  pinMode(PT100_2_pin, INPUT);
  pinMode(FSR_pin, INPUT);

  start_pressure = pressure_read();

  Serial.println("Setup complete.");
}

void loop()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    tryConnectWifi();
  }
  // MAX30102 data
  MAX30102.getHeartbeatSPO2();
  // SHT85 data
  sendSHT85Command(0x2400);
  delay(500);

  uint16_t temperature, humidity;

  if (readSHT85Data(&temperature, &humidity))
  {
    tempC = -45.0 + 175.0 * ((float)temperature / 65535.0);
    humPercent = 100.0 * ((float)humidity / 65535.0);
  }
  else
  {
    Serial.println("SHT85 read failed!");
  }

  print();
  strings_create();

  udp.beginPacket(serverIp, port);
  String message = String(heartreat_read()) + ";" + String(mean_heartrate_read()) + ";" + String(SPO2_read()) + ";" + String(tempC) + ";" + String(humPercent) + ";" + String(pressure_read()) /*+ " ;" + String(maxlipo.cellPercent(), 1)*/ + " ;" + pressure_sting + " ;" + heart_string + " ;" + SPO2_string + " ;" + temp_string + " ;" + hum_string;
  udp.println(message);
  udp.endPacket();
  delay(4000); // Delay between measurements
}
