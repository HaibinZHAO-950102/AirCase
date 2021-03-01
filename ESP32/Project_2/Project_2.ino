
#include "AS726X.h"
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <Arduino.h>
#include <SoftwareSerial.h>
#include "BluetoothSerial.h"
#include "MHZ19.h"

#define RX_PIN 16
#define TX_PIN 17  
#define SEALEVELPRESSURE_HPA (1013.25)
#define BAUDRATE 4800 

SoftwareSerial mySerial(RX_PIN, TX_PIN);

Adafruit_BME680 bme;
AS726X spectrum;
MHZ19 myMHZ19; 
BluetoothSerial SerialBT;

const int venpin = 26, lightpin = 33, lightcontrol = 32;
int venval = 0, lightval;
int sensor[14];

void setup() {
  Wire.begin();
  Serial.begin(115200);
  
  spectrum.begin();

  //while (!Serial);
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

//  SerialBT.begin("Air Quality Monitoring"); //Bluetooth device name
//  Serial.println("The device started, now you can pair it with bluetooth!");
}

void loop() {
  spectrum.takeMeasurements();

  lightval = analogRead(lightpin);
  //Serial.println(lightval);
  sensor[1] = lightval;

  if (sensor[1] > 1000){
    digitalWrite(lightcontrol, HIGH);
    delay(100);
    venval = analogRead(venpin);
    
    sensor[0] = venval;
    sensor[2] = bme.temperature;
    sensor[3] = bme.pressure / 100.0;
    sensor[4] = bme.humidity;
    sensor[5] = bme.gas_resistance / 1000.0;
    sensor[6] = bme.readAltitude(SEALEVELPRESSURE_HPA);
    sensor[7] = myMHZ19.getCO2();
    sensor[8] = spectrum.getCalibratedRed();
    sensor[9] = spectrum.getCalibratedGreen();
    sensor[10] = spectrum.getCalibratedBlue();
    sensor[11] = spectrum.getCalibratedViolet();
    sensor[12] = spectrum.getCalibratedOrange();
    sensor[13] = spectrum.getCalibratedYellow();

    Serial.print("DZ \t LS \t T \t D \t F \t L \t A \t CO2 \t R \t G \t B \t V \t O \t Y \n");
    for(int i = 0; i<14; i++){
      SerialBT.print(sensor[i]);
      Serial.print(sensor[i]);
      SerialBT.print('\n');
      Serial.print("\t");
      }
      Serial.print("\n");
      SerialBT.write('\n');
    }
        
    else {digitalWrite(lightcontrol, LOW);}
    
  delay(1000);

}
