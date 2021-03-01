
#include "AS726X.h"
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <Arduino.h>

#include <SoftwareSerial.h>
#include "BluetoothSerial.h"
#include "MHZ19.h"

#define I2C_SDA 21
#define I2C_SCL 22
#define RX_PIN 16
#define TX_PIN 17  
#define SEALEVELPRESSURE_HPA (1013.25)
#define BAUDRATE 4800

SoftwareSerial mySerial(RX_PIN, TX_PIN);

Adafruit_BME680 bme;
AS726X sensor;
MHZ19 myMHZ19; 

void setup() {
  Wire.begin();
  Serial.begin(115200);
  
  sensor.begin();

  while (!Serial);
  Serial.println(F("BME680 test"));

  if (!bme.begin(0x77)) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1);
  }

  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms

  mySerial.begin(BAUDRATE); 
  myMHZ19.begin(mySerial); 
  myMHZ19.autoCalibration(false);
}

void loop() {
  sensor.takeMeasurements();

  
    //Visible readings
    Serial.print(" Reading: Red[");
    Serial.print(sensor.getCalibratedRed(), 0);
    Serial.print("] G[");
    Serial.print(sensor.getCalibratedGreen(), 0);
    Serial.print("] B[");
    Serial.print(sensor.getCalibratedBlue(), 0);
    Serial.print("] V[");
    Serial.print(sensor.getCalibratedViolet(), 0);
    Serial.print("] O[");
    Serial.print(sensor.getCalibratedOrange(), 0);
    Serial.print("] Y[");
    Serial.print(sensor.getCalibratedYellow(), 0);
 

  Serial.print("]\n");

if (! bme.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
  Serial.print("Temperature = ");
  Serial.print(bme.temperature);
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(bme.pressure / 100.0);
  Serial.println(" hPa");

  Serial.print("Humidity = ");
  Serial.print(bme.humidity);
  Serial.println(" %");

  Serial.print("Gas = ");
  Serial.print(bme.gas_resistance / 1000.0);
  Serial.println(" KOhms");

  Serial.print("Approx. Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  Serial.println();
  delay(2000);

}
