//bmp280 I2C
//esp32 SCL 22, SDA 21
#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp;
unsigned bmpStatus;

float temperature = 0;
float pressure = 0;
float approxAltitude = 0;

//mq135
#include <MQUnifiedsensor.h>

#define Board ("ESP-32")
#define Pin (2)
#define Type ("MQ-135")
#define Voltage_Resolution (3.3) // 3V3 <- IMPORTANT. Source: https://randomnerdtutorials.com/esp32-adc-analog-read-arduino-ide/
#define ADC_Bit_Resolution (12) // ESP-32 bit resolution. Source: https://randomnerdtutorials.com/esp32-adc-analog-read-arduino-ide/
#define RatioMQ135CleanAir 3.6 //RS / R0 = 3.6 ppm  

MQUnifiedsensor MQ135(Board, Voltage_Resolution, ADC_Bit_Resolution, Pin, Type);

//pm2.5
int measurePin = 35;
int ledPower = 5;

unsigned int samplingTime = 280;
unsigned int deltaTime = 40;
unsigned int sleepTime = 9680;

float voMeasured = 0;
float calcVoltage = 0;
float dustDensity = 0;

void setup() {
  Serial.begin(9600);

  //bmp280
  pinMode(ledPower,OUTPUT);  
  bmpStatus = bmp.begin(0x76);
  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
                  
                  
  //mq135 setting math model to calculate the PPM concentration and the value of constants & calibration
  MQ135.setRegressionMethod(1); //_PPM =  a*ratio^b
  MQ135.init(); 
  Serial.print("Calibrating please wait.");
  float calcR0 = 0;
  for(int i = 1; i<=10; i ++)
  {
    MQ135.update();
    calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
    Serial.print(".");
  }
  MQ135.setR0(calcR0/10);
  Serial.println("  done!.");
  if(isinf(calcR0)) {Serial.println("Warning: Conection issue, R0 is infinite (Open circuit detected) please check your wiring and supply"); while(1);}
  if(calcR0 == 0){Serial.println("Warning: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply"); while(1);}

}

void loop() {
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
  
  // linear eqaution taken from http://www.howmuchsnow.com/arduino/airquality/
  // https://www.elecrow.com/wiki/index.php?title=Dust_Sensor-_GP2Y1010AU0F
  // Chris Nafis (c) 2012
  dustDensity = 0.17 * calcVoltage - 0.1;
  
//  Serial.print("Dust Density = ");
//  Serial.print(dustDensity); // unit: mg/m3
//  Serial.println(" mg/m3");

  // ug/mg3
  Serial.print("Dust Density = ");
  Serial.print(dustDensity * 1000); // unit: mg/m3
  Serial.println(" ug/m3");
  
  //bmp280
  if (bmpStatus){
    temperature = bmp.readTemperature();
    pressure = bmp.readPressure();
    approxAltitude = bmp.readAltitude(1013.25); /* Adjusted to local forecast! */
  }
  
  Serial.print(F("Temperature = "));
  Serial.print(temperature);
  Serial.println(" *C");

  Serial.print(F("Pressure = "));
  Serial.print(pressure);
  Serial.println(" Pa");

  Serial.print(F("Approx altitude = "));
  Serial.print(approxAltitude); 
  Serial.println(" m");

  /*
  ----------MQ135----------
    Exponential regression:
  GAS      | a      | b
  CO       | 605.18 | -3.937  
  Alcohol  | 77.255 | -3.18 
  CO2      | 110.47 | -2.862
  Toluen  | 44.947 | -3.445
  NH4      | 102.2  | -2.473
  Aceton  | 34.668 | -3.369
  */

  MQ135.update();
  
  // Configure the equation to calculate CO concentration value
  MQ135.setA(605.18); MQ135.setB(-3.937);
  float CO = MQ135.readSensor();
  
  //Configure the equation to calculate Alcohol concentration value
  MQ135.setA(77.255); MQ135.setB(-3.18);
  float Alcohol = MQ135.readSensor();

  // Configure the equation to calculate CO2 concentration value
  MQ135.setA(110.47); MQ135.setB(-2.862); 
  float CO2 = MQ135.readSensor();

  // Configure the equation to calculate Toluen concentration value
  MQ135.setA(44.947); MQ135.setB(-3.445); 
  float Toluen = MQ135.readSensor();

  // Configure the equation to calculate NH4 concentration value
  MQ135.setA(102.2 ); MQ135.setB(-2.473);
  float NH4 = MQ135.readSensor();

  // Configure the equation to calculate Aceton concentration value
  MQ135.setA(34.668); MQ135.setB(-3.369);
  float Aceton = MQ135.readSensor();
  
  Serial.print("CO = "); Serial.println(CO); 
  Serial.print("Alcohol = "); Serial.println(Alcohol);
  /*
  Motivation:
  We have added 400 PPM because when the library is calibrated it assumes the current state of the
  air as 0 PPM, and it is considered today that the CO2 present in the atmosphere is around 400 PPM.
  https://www.lavanguardia.com/natural/20190514/462242832581/concentracion-dioxido-cabono-co2-atmosfera-bate-record-historia-humanidad.html
  Note: 400 Offset for CO2 source: https://github.com/miguel5612/MQSensorsLib/issues/29
  */
  Serial.print("CO2 = "); Serial.println(CO2 + 400); 
  Serial.print("Toluen = "); Serial.println(Toluen); 
  Serial.print("NH4 = "); Serial.println(NH4); 
  Serial.print("Aceton = "); Serial.println(Aceton);
  Serial.println();
 
  delay(1000);
}
