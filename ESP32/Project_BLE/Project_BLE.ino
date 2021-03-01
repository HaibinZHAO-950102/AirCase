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

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;

#define SERVICE_UUID           "4fcf473e-6469-11eb-ae93-0242ac130002" // UART service UUID
#define CHARACTERISTIC_UUID_RX "4fcf473e-6469-11eb-ae93-0242ac130002"
#define CHARACTERISTIC_UUID_TX "4fcf473e-6469-11eb-ae93-0242ac130002"
String rxload="BlackWalnutLabs";

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };
    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();
      if (rxValue.length() > 0) {
        rxload="";
        for (int i = 0; i < rxValue.length(); i++)
         {
          rxload +=(char)rxValue[i];
          Serial.print(rxValue[i]);
         }
         Serial.println("");
      }
    }
};

void setupBLE(String BLEName){
  const char *ble_name=BLEName.c_str();
  BLEDevice::init(ble_name);
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX,BLECharacteristic::PROPERTY_NOTIFY);
  pCharacteristic->addDescriptor(new BLE2902());
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_RX,BLECharacteristic::PROPERTY_WRITE);
  pCharacteristic->setCallbacks(new MyCallbacks());
  pService->start();
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
}

const int venpin = 26, lightpin = 33, lightcontrol = 32;
int venval = 0, lightval;
int sensor[8];
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
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
  // put your main code here, to run repeatedly:
  
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
    sensor[7] = myMHZ19.getCO2()-150;

    String sendtext = "s,";
    
    Serial.print("LS \t T \t D \t F \t L \t A \t CO2 \n");
    for(int i = 1; i<8; i++){
      Serial.print(sensor[i]);
      Serial.print("\t");
      sendtext += sensor[i];
      sendtext += ",";
      }
      Serial.print("\n");
      sendtext += "e\n";
      const char *newValue=sendtext.c_str();
      pCharacteristic->setValue(newValue);
      pCharacteristic->notify();
    }
    else {digitalWrite(lightcontrol, LOW);}
    
  delay(1000);
  
}
