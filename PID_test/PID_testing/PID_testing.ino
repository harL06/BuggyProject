#define L_HALL 0  // Left wheel sensor connected to digital pin 0
#define R_HALL 1  // Right wheel sensor connected to digital pin 1

volatile int leftCount = 0;  // Stores left wheel pulses
volatile int rightCount = 0; // Stores right wheel pulses

void leftISR() {
  leftCount++;  // Increment count when magnet passes
}

void rightISR() {
  rightCount++; // Increment count when magnet passes
}

void setup() {
  Serial.begin(9600);
  
  pinMode(L_HALL, INPUT_PULLUP);  // Enable internal pull-up resistor
  pinMode(R_HALL, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(L_HALL), leftInterrupt, FALLING);  // Trigger on falling edge
  attachInterrupt(digitalPinToInterrupt(R_HALL), rightInterrupt, FALLING);
}

void loop() {
  Serial.print("Left wheel count: ");
  Serial.print(leftCount);
  Serial.print("\tRight wheel count: ");
  Serial.println(rightCount);
  
  delay(500);  // Print data every 500ms
}
