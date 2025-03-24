// Gold Challenge Group X14

//---HuskyLens Setup---//
#include "HUSKYLENS.h"
HUSKYLENS huskylens;

//---WiFi Setup---//
#include <WiFiS3.h>
char ssid[] = "X14_custom_wifi";
char pass[] = "X14iscool";
WiFiServer server(5180);

//--- WiFi Connecting LED Matrix --- //
#include "Arduino_LED_Matrix.h"
ArduinoLEDMatrix matrix;

byte full_wifi[8][12] = {
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0 },
  { 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0 },
  { 0, 1, 0, 0, 1, 1, 1, 0, 0, 1, 0, 0 },
  { 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
};

byte one_wifi[8][12] = {
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
};

byte two_wifi[8][12] = {
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
};

byte three_wifi[8][12] = {
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0 },
  { 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0 },
  { 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
};

byte blank[8][12] = {
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
};

//---Arduino pin definitions---//
#define L_EYE 7         //left IR Sensor
#define R_EYE 8         //right IR Sensor

#define ENA 10          //left speed pin
#define ENB 11          //right speed pin

#define L_MOTOR_IN1 5   //left motor pin 5
#define L_MOTOR_IN2 6   //left motor pin 6
#define R_MOTOR_IN1 12  //right motor pin 12
#define R_MOTOR_IN2 4  //right motor pin 4

#define TRIG_PIN 3  // Trig pin connected to D3
#define ECHO_PIN 2 // Echo pin connected to D2

#define L_HALL 0  // Trig pin connected to D0
#define R_HALL 1 // Echo pin connected to D1

// Variables for IR sensor states
int prev_left = HIGH;
int prev_right = HIGH;

// Variables for HALL Sensor Interrupts
volatile int Lcount = 0; 
volatile int Rcount = 0;
// Variables for Calculating Distance Travelled
const float WHEEL_RAD = 3; //cm
const float WHEEL_CIRCUM =  PI * WHEEL_RAD * 2;
// Variables for Calculating Wheel Speed
long prevTime = millis(); 
long stoppedTime; // Used to offset time spent at zero speed
long goTime;
long timeStoppedFor;
bool stopped = false;

// Used for PID error tracking and goal speed setting for both wheels
float set_goal_speed = 15;
float goalSpeedL = 15;
float goalSpeedR = 15;
float errorSpeedL = 0;
float cumErrorSpeedL = 160;
float errorSpeedR = 0;
float cumErrorSpeedR = 160;


// Zieglar-Nichols Testing Data
const float tu = 3; // period time (seconds)
const float ku = 6.2; 
// PID constants
const float kp = 0.25 * ku;
const float ki = (0.165 * ku)/ tu;
const float kd = 0; // Harun recommends just killing D coefficient for our use case

//---Driving Functions---//
void Forward(int speedL, int speedR){
  goalSpeedL = set_goal_speed;
  goalSpeedR = set_goal_speed;
  analogWrite(ENA, speedL);   
  analogWrite(ENB, speedR);  
  digitalWrite(L_MOTOR_IN1, HIGH);
  digitalWrite(L_MOTOR_IN2, LOW);
  digitalWrite(R_MOTOR_IN1, HIGH);
  digitalWrite(R_MOTOR_IN2, LOW);
}
// right wheel is the only one moving
void Left(int turnR){
  goalSpeedL = 0;
  goalSpeedR = set_goal_speed;
  analogWrite(ENB, turnR);
  digitalWrite(R_MOTOR_IN1, HIGH);
  digitalWrite(R_MOTOR_IN2, LOW);
  digitalWrite(L_MOTOR_IN1, LOW);
  digitalWrite(L_MOTOR_IN2, LOW);
}
// left wheel is the only one moving
void Right(int turnL){
  goalSpeedR = 0;
  goalSpeedL = set_goal_speed;
  analogWrite(ENA, turnL);  
  digitalWrite(L_MOTOR_IN1, HIGH);
  digitalWrite(L_MOTOR_IN2, LOW);
  digitalWrite(R_MOTOR_IN1, LOW);
  digitalWrite(R_MOTOR_IN2, LOW);
}
// Used to track time spent stopped to not throw off speed measurements (it would read zero distance for a long time)
void Go(){
  goTime = millis();
  stopped = false;
  timeStoppedFor = goTime - stoppedTime;
}

void Stop(){
  stoppedTime = millis();
  stopped = true;
  digitalWrite(L_MOTOR_IN1, LOW);
  digitalWrite(L_MOTOR_IN2, LOW);
  digitalWrite(R_MOTOR_IN1, LOW);
  digitalWrite(R_MOTOR_IN2, LOW);
}

//---UltraSonic Sensor--//
float US_Pulse(){
  long duration;
  float dist;
  // Send a short pulse to trigger pin
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  // Read the pulse duration on the echo pin
  duration = pulseIn(ECHO_PIN, HIGH); 
  // Convert duration to distance in cm
  dist = (duration * 0.0343) / 2; // Speed of sound = 0.034 cm/µs (divide by 2 for round-trip)
  return dist;
}
// This takes a bunch of US pulses and takes the average
float getFilteredDistance() {
    float sum = 0;
    int samples = 3;  // Number of readings to average
    float pulse = 0;
    for (int i = 0; i < samples; i++) {
        pulse = US_Pulse();
        if (pulse != -1) sum += US_Pulse();
        delay(10);  // Small delay between readings
    }
    return sum / samples;  // Return the average distance
}

//---Speed Tracking Functions--//
float calculateDistanceTravelledLeft(){
  float wheel_rotations = (float)Lcount / 4; // 4 counts for every 1 wheel rotation
  return (WHEEL_CIRCUM * wheel_rotations);
}

float calculateDistanceTravelledRight(){
  float wheel_rotations = (float)Rcount / 4; // 4 counts for every 1 wheel rotation
  return (WHEEL_CIRCUM * wheel_rotations);
}

double calculateCurrentLeftSpeed(float elapsedTime) {
    static double prevDistanceL = 0; // Static means value is retained between function calls
    double currentDistanceL = calculateDistanceTravelledLeft();
    float elapsedDistance = currentDistanceL - prevDistanceL;
    double speedL = elapsedDistance / elapsedTime;
    prevDistanceL = currentDistanceL;
    return speedL;
}

double calculateCurrentRightSpeed(float elapsedTime) { 
    static double prevDistanceR = 0; // Static means value is retained between function calls
    double currentDistanceR = calculateDistanceTravelledRight();
    float elapsedDistance = currentDistanceR - prevDistanceR;
    double speedR = elapsedDistance / elapsedTime;
    prevDistanceR = currentDistanceR;
    return speedR;
}

//---PID Functions, does each wheel individually, they track their own seperate errors--//
double computePIDL(double input, float elapsedTime){
    static float prevErrorSpeed = 0;
    errorSpeedL = goalSpeedL - input;
    cumErrorSpeedL += errorSpeedL * elapsedTime;
    float rateError = (errorSpeedL - prevErrorSpeed)/elapsedTime;
    double out = kp*errorSpeedL + ki*cumErrorSpeedL + kd*rateError;
    prevErrorSpeed = errorSpeedL;
    return out;
}

double computePIDR(double input, float elapsedTime){
    static float prevErrorSpeed = 0;
    errorSpeedR = goalSpeedR - input;
    cumErrorSpeedR += errorSpeedR * elapsedTime;
    float rateError = (errorSpeedR - prevErrorSpeed)/elapsedTime;
    double out = kp*errorSpeedR + ki*cumErrorSpeedR + kd*rateError;
    prevErrorSpeed = errorSpeedR;
    return out;
}

//---Interupts to track HALL sensor readings--//
void leftInterrupt() {Lcount++;}
void rightInterrupt() {Rcount++;}


void setup() {
  Serial.begin(9600);
  matrix.begin();

  // Wifi Connection - Attempt to connect
  Serial.print("Connecting to WiFi...");
  matrix.renderBitmap(one_wifi, 8, 12);
  delay(200);
  matrix.renderBitmap(two_wifi, 8, 12);
  delay(200);
  matrix.renderBitmap(three_wifi, 8, 12);
  delay(200);
  // Creates WiFi access point
  WiFi.beginAP(ssid, pass);
  matrix.renderBitmap(full_wifi, 8, 12);
  Serial.println("\nConnected to WiFi");
  // Get and print the IP address
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  server.begin();
  
  //IR Sensors send us signals, we send motors signals
  pinMode(L_EYE, INPUT);
  pinMode(R_EYE, INPUT);
  pinMode(L_MOTOR_IN1, OUTPUT);
  pinMode(L_MOTOR_IN2, OUTPUT);
  pinMode(R_MOTOR_IN1, OUTPUT);
  pinMode(R_MOTOR_IN2, OUTPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(L_HALL, INPUT_PULLUP);  // Enable internal pull-up resistor
  pinMode(R_HALL, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(L_HALL), leftInterrupt, FALLING);  // Triggers interupt on falling edge
  attachInterrupt(digitalPinToInterrupt(R_HALL), rightInterrupt, FALLING);
}

void loop() {
  // Initial Wheel Speeds
  int speedL= 100;
  int speedR= 100;

  //--- Start/Stop boolean and Sensor Trigger Timers---//
  bool running = false;
  float US_dist;
  int US_ticker = 0;
  int HALL_ticker = 0;
  
  WiFiClient client = server.available(); 
  if (client) {  // Check if a client has connected
    matrix.renderBitmap(blank, 8, 12);
    delay(200);
    matrix.renderBitmap(full_wifi, 8, 12);
    delay(200);
    matrix.renderBitmap(blank, 8, 12);
    delay(200);
    matrix.renderBitmap(full_wifi, 8, 12);
    delay(200);
    matrix.renderBitmap(blank, 8, 12);
    delay(200);
    Serial.println("Client connected");
    //client.println("Connected to arduino");  

    while (client.connected()) {
      if (client.available()) {

      String receivedData = client.readStringUntil('\n');  // Read full line
      receivedData.trim();  // Remove spaces/newlines
      if (receivedData.length() > 0) {
          char c = receivedData.charAt(0); // First character
          // GO command from processing
          if (c == 'g') {
              Go();
              running = true;
              Forward(speedL, speedR);
          }
          // STOP command from processing
          else if (c == 's') {
              Stop();
              running = false;
          }
          // SET GOAL SPEED
          else if (c == 'k' && receivedData.length() > 1) {
              int finalSpeed = receivedData.substring(1).toInt();  // Extract number
              set_goal_speed = finalSpeed;
              goalSpeedL = goalSpeedR = set_goal_speed;
          }
          // 
          else if (c == 'f') {
              objectFollowingMode = true;
          }
          else if (c == 'n' && receivedData.length() > 1) {
              int finalSpeed = receivedData.substring(1).toInt();  // Extract number
              set_goal_speed = finalSpeed;
              goalSpeedL = goalSpeedR = set_goal_speed;objectFollowingMode = false;
              if (stopped){
                Go();
              }
          }
        }
      }

      //--- Line Following Code---//
      if (running) {
        //take in what the IR Sensor is giving us
        int current_left = digitalRead(L_EYE);
        int current_right = digitalRead(R_EYE);
        
        // Stopping distance (cm)
        int stopping_dist = 15;
        int slow_dist = 25;
        int fast_dist = 35;

        if (objectFollowingMode && US_ticker >= 30){
          float distance = getFilteredDistance();
          // Sends US sensor data to processing
          client.println("US");
          client.println(distance);

          if (distance <=  stopping_dist){
            goalSpeedL = goalSpeedR = set_goal_speed = 0;
            Stop();
          }
          else if (distance >=  slow_dist && distance < fast_dist){
            goalSpeedL = goalSpeedR = set_goal_speed = 15;
            if (stopped){
              Go();
            }
          }
          else if(distance >= fast_dist){
            goalSpeedL = goalSpeedR = set_goal_speed = 20;
            if (stopped){
              Go();
            }
          }
          US_ticker = 0;
        }

        current_left = digitalRead(L_EYE);
        current_right = digitalRead(R_EYE);
        ////--- Readings and Outputs ---//
          //call forward function
          if (current_left == HIGH && current_right == HIGH ) { 
            Forward(speedL, speedR);
            }
          if (current_left == LOW && current_right == HIGH ) { 
            Left(speedR);
            current_left = digitalRead(L_EYE);
            current_right = digitalRead(R_EYE);
          }
          if (current_left == HIGH && current_right == LOW ) { 
          Right(speedL); 
          current_left = digitalRead(L_EYE);
          current_right = digitalRead(R_EYE);
          }
          if (current_left == LOW && current_right == LOW ) {
          Forward(speedL, speedR);
          }
        // Update the last sensor states for next check
        prev_left = current_left;
        prev_right = current_right;

        US_ticker += 1;
        delay(10);  //wait a second
        
        if (HALL_ticker >= 30){
          long currentTime = millis();
          float elapsedTime = (currentTime - prevTime - timeStoppedFor) / 1000.0; // Convert to seconds
          if (timeStoppedFor != 0){
            timeStoppedFor = 0;
          }
          // Sends distance travelled data to processing
          client.println("HALL");

          float leftSpeed = calculateCurrentLeftSpeed(elapsedTime);
          float rightSpeed = calculateCurrentRightSpeed(elapsedTime);

          speedL = computePIDL(leftSpeed, elapsedTime);
          if (speedL > 255) speedL = 255;
          if (speedL < 0) speedL = 0;
          speedR = computePIDR(rightSpeed, elapsedTime);
          if (speedR > 255) speedR = 255;
          if (speedR < 0) speedR = 0;

          // Send data packet to processing
          client.println("speed_packet");
          String speed_data = String(round(leftSpeed)) + "," + String(round(rightSpeed)) + "," + String(speedL) + "," + String(speedR) + "," + String(set_goal_speed);
          client.println(speed_data);  // Send as a single packet
          Serial.println(set_goal_speed);
          Serial.print(goalSpeedL);
          Serial.println(goalSpeedR);
          HALL_ticker = 0;

          prevTime = currentTime; // Update time reference
        }
        HALL_ticker += 1;
      }
      if (!running) {continue;} // Do Nothing if running bool is false
    }
  }

}
