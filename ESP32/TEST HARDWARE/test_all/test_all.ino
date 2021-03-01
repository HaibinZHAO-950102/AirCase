#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <Arduino.h>
#include "MHZ19.h"                                        
#include <SoftwareSerial.h>                                // Remove if using HardwareSerial


#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10
#define RX_PIN 16                                          // Rx pin which the MHZ19 Tx pin is attached to
#define TX_PIN 17                                          // Tx pin which the MHZ19 Rx pin is attached to

#define BAUDRATE 9600                                      // Device to MH-Z19 Serial baudrate (should not be changed)
#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME680 bme; // I2C
MHZ19 myMHZ19;                                             // Constructor for library
SoftwareSerial mySerial(RX_PIN, TX_PIN);                   // (Uno example) create device to MH-Z19 serial

const int venpin = 26, lightpin = 33;
int venval = 0, lightval;
int sensor[8];
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(venpin, INPUT);
  pinMode(lightpin, INPUT);
   while (!Serial);
  Serial.println(F("BME680 test"));

  if (!bme.begin()) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1);}
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
  delay(1000);

  mySerial.begin(BAUDRATE);                               // (Uno example) device to MH-Z19 serial start   
    myMHZ19.begin(mySerial);                                // *Serial(Stream) refence must be passed to library begin(). 

    myMHZ19.autoCalibration();                              // Turn auto calibration ON (OFF autoCalibration(false))
}

void loop() {
  // put your main code here, to run repeatedly:
  venval = analogRead(venpin);
  lightval = analogRead(lightpin);
  sensor[0] = venval;
  sensor[1] = lightval;
  sensor[2] = bme.temperature;
  sensor[3] = bme.pressure / 100.0;
  sensor[4] = bme.humidity;
  sensor[5] = bme.gas_resistance / 1000.0;
  sensor[6] = bme.readAltitude(SEALEVELPRESSURE_HPA);
  sensor[7] = myMHZ19.getCO2();

  Serial.print("DZ \t LS \t T \t D \t F \t L \t A \t CO2 \n");
  for(int i = 0; i<8; i++){
    Serial.print(sensor[i]);
    Serial.print("\t");}
  Serial.print("\n");
  delay(1000);
}
