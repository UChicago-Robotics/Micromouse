// Transmitter: 220 Ohms, Receiver: 7MOhm
//https://www.adafruit.com/product/2831
const int digitalPin = 3;
const int analogPin = A0;
const int pulseIntervalMicros = 250000; // 4 times per second

void setup() {
  pinMode(digitalPin, OUTPUT);
  digitalWrite(digitalPin, HIGH);
  Serial.begin(9600);
}

void loop() {
  delayMicroseconds(100); // Briefly set pin 3 high
  int sensorValue = analogRead(analogPin);
  Serial.print(1024);
  Serial.print(" ");
  Serial.println(sensorValue); // Print sensor value to console
}
