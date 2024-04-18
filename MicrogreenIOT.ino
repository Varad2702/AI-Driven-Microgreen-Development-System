#include <WiFi.h>
#include <HTTPClient.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <InfluxDbClient.h>
#include <InfluxDbCloud.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "time.h"

#define WIFI_SSID "Your_WiFi_SSID"
#define WIFI_PASSWORD "Your_WiFi_Password"
#define INFLUXDB_URL "https://eu-central-1-1.aws.cloud2.influxdata.com"
#define INFLUXDB_TOKEN "Your_InfluxDB_Token_Here"
#define INFLUXDB_ORG "Your_InfluxDB_Org_Here"
#define INFLUXDB_BUCKET "ESP32_DHT11"

#define DHTPIN 27
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

#define PH_SENSOR_PIN 32
#define TDS_SENSOR_PIN 33

#define SD_CS_PIN 5

// RGB LED Pins
const int redPin = 25;
const int greenPin = 26;
const int bluePin = 27;

// Define LEDC parameters
const int freq = 5000;
const int redChannel = 0;
const int greenChannel = 1;
const int blueChannel = 2;
const int resolution = 8;

InfluxDBClient client(INFLUXDB_URL, INFLUXDB_ORG, INFLUXDB_BUCKET, INFLUXDB_TOKEN, InfluxDbCloud2CACert);

void setup() {
  Serial.begin(115200);
  dht.begin();

  // Initialize SD card
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("Card Mount Failed");
    return;
  }
  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    return;
  }

  // Connect to WiFi and NTP
  setupWiFiNTP();

  // Initialize RGB LED
  ledcSetup(redChannel, freq, resolution);
  ledcSetup(greenChannel, freq, resolution);
  ledcSetup(blueChannel, freq, resolution);
  ledcAttachPin(redPin, redChannel);
  ledcAttachPin(greenPin, greenChannel);
  ledcAttachPin(bluePin, blueChannel);

  analogReadResolution(10); // For pH Sensor
}

void loop() {
  delay(2000); // Wait a few seconds between measurements

  // Check if there is any Serial input
  checkSerial();

  // Sensor readings and data logging
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();
  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  float phValue = readPH();
  float tdsValue = readTDS();

  // Save to SD Card
  saveDataToSD(temperature, humidity, phValue, tdsValue);

  // Output to Serial Monitor
  outputSensorData(temperature, humidity, phValue, tdsValue);

  // Write data to InfluxDB
  writeDataToInfluxDB(temperature, humidity, phValue, tdsValue);
}

void saveDataToSD(float temperature, float humidity, float phValue, float tdsValue) {
  time_t now = time(nullptr);
  struct tm* timeinfo = localtime(&now);
  char folderName[32];
  sprintf(folderName, "/sensorData_%04d-%02d-%02d", timeinfo->tm_year + 1900, timeinfo->tm_mon + 1, timeinfo->tm_mday);
  
  if (!SD.exists(folderName)) {
    SD.mkdir(folderName);
  }
  
  char fileName[64];
  sprintf(fileName, "%s/sensorData_%04d-%02d-%02d.csv", folderName, timeinfo->tm_year + 1900, timeinfo->tm_mon + 1, timeinfo->tm_mday);
  
  File file = SD.open(fileName, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  
  file.print(temperature);
  file.print(",");
  file.print(humidity);
  file.print(",");
  file.print(phValue);
  file.print(",");
  file.println(tdsValue);
  file.close();
  Serial.println("Data saved to SD card.");
}

void outputSensorData(float temperature, float humidity, float phValue, float tdsValue) {
  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.print("%\t");
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println("°C");
  Serial.print("pH Value: ");
  Serial.println(phValue, 2);
  Serial.print("TDS Value: ");
  Serial.println(tdsValue, 2);
}

void writeDataToInfluxDB(float temperature, float humidity, float phValue, float tdsValue) {
  Point sensor("environment");
  sensor.addField("temperature", temperature);
  sensor.addField("humidity", humidity);
  sensor.addField("pH", phValue);
  sensor.addField("TDS", tdsValue);

  if (!client.writePoint(sensor)) {
    Serial.print("InfluxDB write failed: ");
    Serial.println(client.getLastErrorMessage());
  }
}

void setupWiFiNTP() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.println("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to WiFi");
  
  // Initialize NTP
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  Serial.println("Waiting for NTP time sync");
  time_t nowSecs = time(nullptr);
  while (nowSecs < 8 * 3600 * 2) {
    delay(500);
    Serial.print(".");
    nowSecs = time(nullptr);
  }
  Serial.println("Time synchronized");
}

void checkSerial() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim(); // Trim whitespace
    int redValue, greenValue, blueValue;
    if (parseInput(input, redValue, greenValue, blueValue)) {
      setColor(redValue, greenValue, blueValue);
    } else {
      Serial.println("Invalid format. Use 'R:xx,G:yy,B:zz'.");
    }
  }
}

void setColor(int redValue, int greenValue, int blueValue) {
  // Write values to the RGB pins
  ledcWrite(redChannel, redValue);
  ledcWrite(greenChannel, greenValue);
  ledcWrite(blueChannel, blueValue);
}

bool parseInput(String input, int &red, int &green, int &blue) {
  // Parsing logic to extract RGB values from Serial input
  if (sscanf(input.c_str(), "R:%d,G:%d,B:%d", &red, &green, &blue) == 3) {
    return true;
  } else {
    return false;
  }
}

float readTDS() {
  int sensorValue = analogRead(TDS_SENSOR_PIN);
  float voltage = sensorValue * 3.3 / 1023.0; // Convert analog reading to Voltage
  float tdsValue = (133.42 * voltage * voltage * voltage - 255.86 * voltage * voltage + 857.39 * voltage) * 0.5; // TDS Formula
  return tdsValue;
}

float readPH() {
  unsigned long int avgValue = 0;
  int buf[10], temp;
  
  for (int i = 0; i < 10; i++) {
    buf[i] = analogRead(PH_SENSOR_PIN);
    delay(10);
  }
  for (int i = 0; i < 9; i++) {
    for (int j = i + 1; j < 10; j++) {
      if (buf[i] > buf[j]) {
        temp = buf[i];
        buf[i] = buf[j];
        buf[j] = temp;
      }
    }
  }
  for (int i = 2; i < 8; i++) avgValue += buf[i];
  float phValue = (float)avgValue * 5.0 / 1024 / 6; // Adjust the 5.0V reference voltage to your actual AREF
  phValue = 3.5 * phValue; // Convert the millivolt into pH value
  return phValue;
}

