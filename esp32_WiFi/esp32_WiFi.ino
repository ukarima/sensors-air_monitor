//WIFI & NTP setup
#include <WiFi.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <ArduinoJson.h>
#include "WiFiCredentials.h"

#define SerialMon Serial

const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWD;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);
String formattedDate;
String dateStamp;
String timeStamp;

WiFiClient client;

// MQTT setup
#include <PubSubClient.h>
PubSubClient mqtt(client);
const char* broker = MQTT_BROKER;
const char* topicPub = MQTT_TOPIC_PUB;
const char* topicSub = MQTT_TOPIC_SUB;

const unsigned long period = 2*1000L; // do loop every 60s
unsigned long startMillis;
unsigned long currentMillis;
uint32_t lastReconnectAttempt = 0;

//mq135
#include "MQ135.h"
int airQualityPin = 35;

//bmp280 I2C
//esp32 SCL 22, SDA 21
#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp;
unsigned bmpStatus;

float temperature, pressure, approxAltitude = 0;

//pm2.5
int measurePin = 34;
int ledPower = 26;

unsigned int samplingTime = 280;
unsigned int deltaTime = 40;
unsigned int sleepTime = 9680;

float voMeasured, calcVoltage, dustDensity = 0;

void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  SerialMon.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    SerialMon.print('.');
    delay(1000);
  }
  SerialMon.println(WiFi.localIP());
  SerialMon.print("Connected to ");
  SerialMon.println(ssid);
  digitalWrite(19, LOW);  // turn the LED off
}

void mqttCallback(char* topic, byte* payload, unsigned int len) {
//  SerialMon.print("Message arrived [");
//  SerialMon.print(topic);
//  SerialMon.print("]: ");
  SerialMon.print("[ARRIVED] ");
  SerialMon.write(payload, len);
  SerialMon.println();
}

boolean mqttConnect() {
  SerialMon.print("Connecting to ");
  SerialMon.print(broker);

  // Connect to MQTT Broker
  boolean status = mqtt.connect("");

  if (status == false) {
    SerialMon.println(" fail");
    return false;
  }
  SerialMon.println(" success");
  return mqtt.connected();
}

void setup() {
  SerialMon.begin(115200);
  pinMode(19, OUTPUT); //blue led
  digitalWrite(19, HIGH); // turn the LED on
  delay(10);
  initWiFi();

  timeClient.begin();
  timeClient.setTimeOffset(25200);

  //mq135
  pinMode(airQualityPin, INPUT);

  //bmp280
  pinMode(ledPower,OUTPUT);  
  bmpStatus = bmp.begin(0x76);
  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
                  
  // MQTT setup
  mqtt.setServer(broker, 1883);
  mqtt.setCallback(mqttCallback);

  startMillis = millis();
}

void loop() {
  if (!mqtt.connected()) {
      SerialMon.println("=== MQTT NOT CONNECTED ===");
      // Reconnect every 10 seconds
      uint32_t t = millis();
      if (t - lastReconnectAttempt > 10000L) {
        lastReconnectAttempt = t;
        if (mqttConnect()) {
          lastReconnectAttempt = 0;
        }
      }
      delay(100);
      return;
    }
  currentMillis = millis();
  if (currentMillis - startMillis >= period) {
    startMillis = currentMillis;
    
    //mq135
    MQ135 gasSensor = MQ135(airQualityPin);
    float airQuality = gasSensor.getPPM();
    
    //pm2.5
    digitalWrite(ledPower,LOW); // power on the LED
    delayMicroseconds(samplingTime);
  
    voMeasured = analogRead(measurePin); // read the dust value
  
    delayMicroseconds(deltaTime);
    digitalWrite(ledPower,HIGH); // turn the LED off
    delayMicroseconds(sleepTime);

    // 0 - 3.3V mapped to 0 - 1023 integer values
    // recover voltage
    calcVoltage = voMeasured * (3.3 / 1024.0);
  
    /*
    * linear eqaution taken from http://www.howmuchsnow.com/arduino/airquality/
    * https://www.elecrow.com/wiki/index.php?title=Dust_Sensor-_GP2Y1010AU0F
    * Chris Nafis (c) 2012
    * unit: mg/m3 --> ug/m3 unit = * 1000
    */
    dustDensity = 0.17 * calcVoltage - 0.1;
  
    if (dustDensity < 0) {
      dustDensity = 0;
    }
  
    //bmp280
    if (bmpStatus){
      temperature = bmp.readTemperature();
      pressure = bmp.readPressure();
      approxAltitude = bmp.readAltitude(1013.25); /* Adjusted to local forecast! */
    }
  
    while(!timeClient.update()) {
      timeClient.forceUpdate();
    }
    
    formattedDate = timeClient.getFormattedDate();
    
    // Extract date
    int splitT = formattedDate.indexOf("T");
    dateStamp = formattedDate.substring(0, splitT);
    
    // Extract time
    timeStamp = formattedDate.substring(splitT+1, formattedDate.length()-1);

    StaticJsonDocument<256> JSONBuffer;

    JSONBuffer["date"] = dateStamp;
    JSONBuffer["time"] = timeStamp;
    JSONBuffer["airQuality"] = airQuality; // unit: ug/m3
    JSONBuffer["dustDensity"] = dustDensity * 1000; // unit: ug/m3
    JSONBuffer["temperature"] = temperature; // *C
    JSONBuffer["pressure"] = pressure; // Pa
    JSONBuffer["approxAltitude"] = approxAltitude; // m

    char JSONString[256];
    
    serializeJson(JSONBuffer, JSONString);
    mqtt.publish(topicPub, JSONString);
    SerialMon.print("[SENT] ");
    SerialMon.println(JSONString);
    delay(100);
    mqtt.subscribe(topicSub);
  }
  mqtt.loop();
}
