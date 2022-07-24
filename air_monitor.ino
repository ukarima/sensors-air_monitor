#include <ArduinoJson.h>
#include "credentials.h"

bool connectedStatus = true;

// GSM setup
//#include <HardwareSerial.h>
//HardwareSerial SerialAT(1); 
#include <SoftwareSerial.h> 
SoftwareSerial SerialAT(33, 5);

#define TINY_GSM_MODEM_BG96
#define PWRKEY_PIN 25
#define RESET_PIN 27
#define TXD_PIN 5
#define RXD_PIN 33 
#define SerialMon Serial

int BAUDRATE = 115200;

const char apn[] = APN_NAME;
const char gprsUser[] = "";
const char gprsPass[] = "";

// MQTT setup
const char* broker = MQTT_BROKER;
const char* topicPub = MQTT_TOPIC_PUB;

#include <TinyGsmClient.h>
#include <PubSubClient.h>

TinyGsm modem(SerialAT);
TinyGsmClient client(modem);
PubSubClient mqtt(client);

int counter = 0;
const unsigned long period = 60*1000L; // do loop every 60s
unsigned long startMillis;
unsigned long currentMillis;
String csq;
uint32_t lastReconnectAttempt = 0;

//bmp280 I2C
//esp32 SCL 22, SDA 21
#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp;
unsigned bmpStatus;

float temperature, pressure, approxAltitude = 0;

//mq135
#include "MQ135.h"
int airQualityPin = 35;

//pm2.5
int measurePin = 34;
int ledPower = 26;

unsigned int samplingTime = 280;
unsigned int deltaTime = 40;
unsigned int sleepTime = 9680;

float voMeasured, calcVoltage, dustDensity = 0;

String dateStamp;
String timeStamp;

void mqttCallback(char* topic, byte* payload, unsigned int len) {
  SerialMon.print("Message arrived [");
  SerialMon.print(topic);
  SerialMon.print("]: ");
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

void setup_modem() {
  SerialAT.print("AT\r\n");
  delay(100);
  String input = SerialAT.readString();
  if (input.indexOf("OK") < 0) {
    SerialMon.println("turning on modem");
    pinMode(PWRKEY_PIN, OUTPUT);
    pinMode(RESET_PIN, OUTPUT);
    digitalWrite(RESET_PIN, LOW);
    digitalWrite(PWRKEY_PIN, LOW);
    delay(1000);
    SerialMon.println("Turn ON");
    digitalWrite(PWRKEY_PIN, HIGH);
    delay(1000);
    digitalWrite(PWRKEY_PIN, LOW);
    SerialMon.println("Wait...");
    delay(6000);
  }
  SerialMon.println("already on");
  SerialMon.println("Ready for AT command");

  if (setNwscanmode()) {
    SerialMon.println("set GSM mode success");
  }
  if (setNwscanseq()) {
    SerialMon.println("set GSM as priority success");
  }
 
  SerialMon.println("Initializing modem...");
  modem.restart();

  String modemInfo = modem.getModemInfo();
  SerialMon.print("Modem Info: ");
  Serial.println(modemInfo);
  
  SerialAT.print("AT+CFUN=1\r\n");
  delay(100);
  String input1 = SerialAT.readString();
  SerialMon.println(input1);
  while (!checkSignal()) {
    Serial.print("check signal quality");
    delay(5000);
    counter++;
    if (counter > 20) {
      Serial.println("restart csq");
      ESP.restart();
    }
  }

  Serial.println("Waiting for network...");
  if (!modem.waitForNetwork()) {
    Serial.print(" fail");
    delay(30000);
    String input4 = SerialAT.readString();
    SerialMon.println(input4);
    Serial.println("restart network");
    ESP.restart();
  }
  SerialMon.println(" success");

  if (modem.isNetworkConnected()) {
    SerialMon.println("Network connected");
  } else {
    SerialMon.println("Modem reset");
    Serial.println("restart network2");
    ESP.restart();
  }

  // GPRS connection parameters are usually set after network registration
  Serial.print(F("Connecting to "));
  Serial.println(apn);

  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    SerialMon.println(" fail");
    delay(10000);
    SerialMon.println("ESP reset");
    ESP.restart();
  }
  SerialMon.println(" success");

  if (modem.isGprsConnected()) {
    Serial.println("GPRS connected");
    connectedStatus = false;
    digitalWrite(19, LOW);  // turn the LED off
  }
}

boolean checkSignal() {
  SerialAT.print("AT+CSQ\r\n");
  delay(100);
  csq = SerialAT.readString();
  Serial.println(csq);
  if (csq.indexOf("99,99") > 0) {
    return 0;
  } else {
    return 1;
  }
}

boolean setNwscanmode() {
  SerialAT.print("AT+QCFG=\"nwscanmode\",1,1\r\n");
  delay(100);

  String resp = SerialAT.readString();
  Serial.println(resp);
  if (resp.indexOf("OK") > 0) {
    return 1;
  } else {
    return 0;
  }
}

boolean setNwscanseq() {
  SerialAT.print("AT+QCFG=\"nwscanseq\",01,1\r\n");
  delay(100);

  String resp = SerialAT.readString();
  Serial.println(resp);
  if (resp.indexOf("OK") > 0) {
    return 1;
  } else {
    return 0;
  }
}

void setup() {
  SerialMon.begin(115200);
  pinMode(19, OUTPUT);
  digitalWrite(19, HIGH); // turn the LED on
//  SerialAT.begin(BAUDRATE, SERIAL_8N1, RXD_PIN, TXD_PIN); 
  SerialAT.begin(9600);
  delay(10);
  setup_modem();
  
  // MQTT setup
  mqtt.setServer(broker, 1883);
  mqtt.setCallback(mqttCallback);

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

    // timestamp if NTP doesn't work
    SerialAT.print("AT+CCLK?\r\n");
    delay(100);
    
    String resp = SerialAT.readString();
    int split = resp.indexOf(",");
    
    String getHour = resp.substring(split+1, split+3);
    int hour = getHour.toInt()+7;

    if (hour >= 24) {
      hour -= 24;
    }
    
    dateStamp = resp.substring(split-8, split);
    timeStamp = String(hour) + resp.substring(split+3, split+9); 

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
    SerialMon.println(JSONString);
    mqtt.publish(topicPub, JSONString);
  }

  mqtt.loop();
}
