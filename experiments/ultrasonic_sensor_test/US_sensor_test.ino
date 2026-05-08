#define TRIG_PIN 3  // Connect Trig to digital pin 3
#define ECHO_PIN 2  // Connect Echo to digital pin 2

void setup() {
  Serial.begin(9600);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void loop() {
  float duration;
  float distance;
  
  // Ensure a clean pulse by starting LOW
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  
  // Send a 10µs HIGH pulse to trigger the sensor
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Read the duration of the echo pulse (timeout after 30ms)
  duration = pulseIn(ECHO_PIN, HIGH);
  
  // Check if we received an echo
  if (duration == 0) {
    Serial.println("No echo received - check wiring or sensor power");
  } else {
    // Calculate distance in centimetres
    distance = (duration * 0.0343) / 2;
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
  }
  
  delay(500);
}
