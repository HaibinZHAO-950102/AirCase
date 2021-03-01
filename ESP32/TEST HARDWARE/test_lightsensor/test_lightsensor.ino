const int potPin = 33;
int light = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(potPin, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  light = analogRead(potPin);
  Serial.println(light);
  delay(500);
}
