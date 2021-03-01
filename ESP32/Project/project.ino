#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <Arduino.h>
#include "MHZ19.h"                                        
//#include <SoftwareSerial.h>                                // Remove if using HardwareSerial
#include "BluetoothSerial.h"
#include "AS726X.h"

//#define RX_PIN 16                                          // Rx pin which the MHZ19 Tx pin is attached to
//#define TX_PIN 17                                          // Tx pin which the MHZ19 Rx pin is attached to

//#define BAUDRATE 9600                                      // Device to MH-Z19 Serial baudrate (should not be changed)
#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME680 bme; // I2C
//MHZ19 myMHZ19;                                             // Constructor for library
AS726X spectrum;
//SoftwareSerial mySerial(RX_PIN, TX_PIN);                   // (Uno example) create device to MH-Z19 serial
BluetoothSerial SerialBT;


const int venpin = 26, lightpin = 33, lightcontrol = 32;
int venval = 0, lightval;
int sensor[14];


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  //pinMode(venpin, INPUT);
  //pinMode(lightpin, INPUT);
  //pinMode(lightcontrol, OUTPUT);

  while (!Serial);
  Serial.println(F("BME680 test"));

  spectrum.begin();
  
  //if (!bme.begin(0x77)) {
  //  Serial.println("Could not find a valid BME680 sensor, check wiring!");
  //  while (1);}
    
  //bme.setTemperatureOversampling(BME680_OS_8X);
  //bme.setHumidityOversampling(BME680_OS_2X);
  //bme.setPressureOversampling(BME680_OS_4X);
  //bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  //bme.setGasHeater(320, 150); // 320*C for 150 ms
  
  //mySerial.begin(BAUDRATE);                               // (Uno example) device to MH-Z19 serial start   
  //myMHZ19.begin(mySerial);                                // *Serial(Stream) refence must be passed to library begin(). 
  //myMHZ19.autoCalibration(false);                              // Turn auto calibration ON (OFF autoCalibration(false))

  //SerialBT.begin("Air Quality Monitoring"); //Bluetooth device name
  //Serial.println("The device started, now you can pair it with bluetooth!");
}

void loop() {
  // put your main code here, to run repeatedly:
  spectrum.takeMeasurements();
  
  lightval = analogRead(lightpin);
  Serial.println(lightval);
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
    for(int i = 0; i<13; i++){
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
