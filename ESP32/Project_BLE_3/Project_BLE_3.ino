#include "AS726X.h"
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <Arduino.h>
#include "MHZ19.h"                                        
#include <SoftwareSerial.h>                                // Remove if using HardwareSerial
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <String.h>


#define RX_PIN 16                                          // Rx pin which the MHZ19 Tx pin is attached to
#define TX_PIN 17                                          // Tx pin which the MHZ19 Rx pin is attached to

#define BAUDRATE 9600                                      // Device to MH-Z19 Serial baudrate (should not be changed)
#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME680 bme; // I2C
AS726X spectrum;
MHZ19 myMHZ19;                                             // Constructor for library
SoftwareSerial mySerial(RX_PIN, TX_PIN);                   // (Uno example) create device to MH-Z19 serial

BLECharacteristic *pCharacteristic;
BLECharacteristic *p2Characteristic;
bool deviceConnected = false;

#define SERVICE_UUID            "ca7b1cb0-144c-40f3-a0e6-409c6bd1a300"
#define CHARACTERISTIC_UUID_RX  "87fd265a-6476-11eb-ae93-0242ac130002"
#define CHARACTERISTIC_UUID_1   "5bfdc0be-6476-11eb-ae93-0242ac130002"
#define CHARACTERISTIC_UUID_2   "d2c03edc-64b4-11eb-ae93-0242ac130002"
#define CHARACTERISTIC_UUID_3   "d8eaca70-64b4-11eb-ae93-0242ac130002"
#define CHARACTERISTIC_UUID_4   "dd5fa4ea-64b4-11eb-ae93-0242ac130002"
#define CHARACTERISTIC_UUID_5   "e1441bf4-64b4-11eb-ae93-0242ac130002"
#define CHARACTERISTIC_UUID_6   "84e1aa56-64b5-11eb-ae93-0242ac130002"
#define CHARACTERISTIC_UUID_7   "893ce75a-64b5-11eb-ae93-0242ac130002"

String rxload="BlackWalnutLabs";

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };
    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

//class MyCallbacks: public BLECharacteristicCallbacks {
//    void onWrite(BLECharacteristic *pCharacteristic) {
//      std::string rxValue = pCharacteristic->getValue();
//      if (rxValue.length() > 0) {
//        rxload="";
//        for (int i = 0; i < rxValue.length(); i++)
//         {
//          rxload +=(char)rxValue[i];
//          Serial.print(rxValue[i]);
//         }
//         Serial.println("");
//      }
//    }
//};

void setupBLE(String BLEName){
  const char *ble_name=BLEName.c_str();
  BLEDevice::init(ble_name);
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  
  BLEService *pService = pServer->createService(SERVICE_UUID);
  
  pCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_1, BLECharacteristic::PROPERTY_NOTIFY);
  pCharacteristic->addDescriptor(new BLE2902());
  
  BLECharacteristic *p2Characteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_2, BLECharacteristic::PROPERTY_NOTIFY);
  p2Characteristic->addDescriptor(new BLE2902());

  
  pService->start();
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
}

const int venpin = 26, lightpin = 33, lightcontrol = 32;
int venval = 0, lightval;
int sensor[14];
void setup() {
  Wire.begin();
  Serial.begin(115200);
  
  spectrum.begin();
  
  setupBLE("AirCase");//设置蓝牙名称
  pinMode(venpin, INPUT);
  pinMode(lightpin, INPUT);
  pinMode(lightcontrol, OUTPUT);
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
  
  mySerial.begin(BAUDRATE);                               // (Uno example) device to MH-Z19 serial start   
  myMHZ19.begin(mySerial);                                // *Serial(Stream) refence must be passed to library begin(). 
  myMHZ19.autoCalibration(false);                              // Turn auto calibration ON (OFF autoCalibration(false))


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

    String sendtext1 = "s";
    String sendtext2 = "Hello";
    
    Serial.print("DZ \t LS \t T \t D \t F \t L \t A \t CO2 \t R \t G \t B \t V \t O \t Y \n");
    for(int i = 0; i<14; i++){
      Serial.print(sensor[i]);
      Serial.print("\t");
      sendtext1 += sensor[i];
      sendtext1 += "s";
      }
      Serial.print("\n");
      sendtext1 += "\n";
      Serial.println(sendtext1);

      const char *newValue1 = sendtext1.c_str();
      const char *newValue2 = sendtext2.c_str();
      pCharacteristic->setValue(newValue1);
      pCharacteristic->notify();
      p2Characteristic->setValue(newValue2);
      p2Characteristic->notify();
    }
    else {digitalWrite(lightcontrol, LOW);}
    
  delay(1000);
  
}
