#define controlpin 14
#define readpin 27

int rs = 0;
 const int freq = 30000;
 const int resolution = 8; 
 const int ledChannel = 0;
 
void setup() {
 Serial.begin(9600);
 pinMode(controlpin, OUTPUT);
 pinMode(readpin, INPUT); 

 ledcSetup(ledChannel, freq, resolution);
 //ledcAttachPin(controlpin, ledChannel);

}

void loop() {
  
    //ledcWrite(ledChannel, 200);
    digitalWrite(controlpin, HIGH);
    //rs = analogRead(readpin);
    //Serial.println(rs);
    delay(15);
 
}
