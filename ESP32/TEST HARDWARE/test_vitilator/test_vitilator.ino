const int s = 25;
int vol = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(2400);
  pinMode(s, INPUT);
  pinMode(32, OUTPUT);
  delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(32, HIGH);
  vol = analogRead(s);
  Serial.println(vol);
  delay(100);
}
