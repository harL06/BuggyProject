// Group X14

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

//#define US_IN 1       //US Sensor Inputs
//#define US_Out 1      //US Sensor Outputs

#define L_MOTOR_IN1 5   //left motor pin 5
#define L_MOTOR_IN2 6   //left motor pin 6
#define R_MOTOR_IN1 12  //right motor pin 12
#define R_MOTOR_IN2 4  //right motor pin 4

#define TRIG_PIN 3  // Trig pin connected to D3
#define ECHO_PIN 2 // Echo pin connected to D2

#define L_HALL 0  // Trig pin connected to D0
#define R_HALL 1 // Echo pin connected to D1


volatile int Lcount = 0; // Counts how many times hall sensor picks up left/right wheel
volatile int Rcount = 0;


const float WHEEL_RAD = 3; //cm
const float WHEEL_CIRCUM =  PI * WHEEL_RAD * 2;

static long prevTime = 0; // Global Timing variable for speed calculation

// Used for PID error tracking and goal speed setting
float goalSpeed = 35;
float errorSpeedL = 0;
float cumErrorSpeedL = 0;
float errorSpeedR = 0;
float cumErrorSpeedR = 0;

// PID constants
const float tu = 2; // period time (seconds)
const float ku = 7.5;

const float kp = 0.3 * ku;
const float ki = (1.2 * ku)/ tu;
const float kd = 0.045 * ku * tu;


//---Driving Functions---//
            
//         normal speed, normal speed
void Forward(int speedL, int speedR){
  analogWrite(ENA, speedL);   
  analogWrite(ENB, speedR);  
 
  digitalWrite(L_MOTOR_IN1, HIGH);
  digitalWrite(L_MOTOR_IN2, LOW);
  digitalWrite(R_MOTOR_IN1, HIGH);
  digitalWrite(R_MOTOR_IN2, LOW);
 
}

void Stop(){
  digitalWrite(L_MOTOR_IN1, LOW);
  digitalWrite(L_MOTOR_IN2, LOW);
  digitalWrite(R_MOTOR_IN1, LOW);
  digitalWrite(R_MOTOR_IN2, LOW);
}

//         normal speed, normal speed
void Backward(int speedL, int speedR){
  analogWrite(ENA, speedL);  
  analogWrite(ENB, speedR);
 
  digitalWrite(L_MOTOR_IN1, LOW);
  digitalWrite(L_MOTOR_IN2, HIGH);
  digitalWrite(R_MOTOR_IN1, LOW);
  digitalWrite(R_MOTOR_IN2, HIGH);

}

// right wheel is the only one moving
void Left(int turnR){

  analogWrite(ENB, turnR);
 
  digitalWrite(R_MOTOR_IN1, HIGH);
  digitalWrite(R_MOTOR_IN2, LOW);
  digitalWrite(L_MOTOR_IN1, LOW);
  digitalWrite(L_MOTOR_IN2, LOW);
  
}
// left wheel is the only one moving
void Right(int turnL){
  
  analogWrite(ENA, turnL);  
  
  digitalWrite(L_MOTOR_IN1, HIGH);
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

  // Print the distance to the Serial Monitor
  //Serial.print("Distance: ");
  //Serial.print(dist);
  //Serial.println(" cm");


  // TEST TO SEE IF THIS IS BREAKING THINGS
  //   if (dist > 400) {  // Discard absurd readings (sensor max is ~400 cm)
  //     return -1;  // Return an invalid value
  // }

  return dist;
}


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

    double speedL = elapsedDistance / elapsedTime; // Avoid division by zero

    prevDistanceL = currentDistanceL;

    return speedL;
}

double calculateCurrentRightSpeed(float elapsedTime) { 
    static double prevDistanceR = 0; // Static means value is retained between function calls
    double currentDistanceR = calculateDistanceTravelledRight();

    float elapsedDistance = currentDistanceR - prevDistanceR;

    double speedR = elapsedDistance / elapsedTime; // Avoid division by zero

    prevDistanceR = currentDistanceR;

    return speedR;
}

double computePIDL(double input, float elapsedTime){
    static float prevErrorSpeed = 0;
    errorSpeedL = goalSpeed - input;
    cumErrorSpeedL += errorSpeedL * elapsedTime;
    float rateError = (errorSpeedL - prevErrorSpeed)/elapsedTime;

    Serial.print(goalSpeed);
    Serial.print(",");
    Serial.print(kp*errorSpeedL);
    Serial.print(",");
    Serial.print(ki*cumErrorSpeedL);
    Serial.print(",");
    Serial.print(kd*rateError);
    Serial.print(",");

    double out = kp*errorSpeedL + ki*cumErrorSpeedL + kd*rateError;

    Serial.print(out);
    Serial.print(",");

    prevErrorSpeed = errorSpeedL;
    //Serial.print(out); Serial.print(",");

    return out;
}

double computePIDR(double input, float elapsedTime){
    static float prevErrorSpeed = 0;
    errorSpeedR = goalSpeed - input;
    cumErrorSpeedR += errorSpeedR * elapsedTime;
    float rateError = (errorSpeedR - prevErrorSpeed)/elapsedTime;

    // Serial.print(goalSpeed);
    // Serial.print(",");
    // Serial.print(errorSpeedR);
    // Serial.print(",");

    double out = kp*errorSpeedR + ki*cumErrorSpeedR + kd*rateError;

    prevErrorSpeed = errorSpeedR;
    //Serial.print(out); Serial.print(",");

    return out;
}


//initialises a place to hold the previous IR input
int prev_left = HIGH;
int prev_right = HIGH;



void setup() {
  Serial.begin(9600);
  matrix.begin();

  // Wifi Connection
  // Attempt to connect to WiFi
  Serial.print("Connecting to WiFi...");
  matrix.renderBitmap(one_wifi, 8, 12);
  delay(200);
  matrix.renderBitmap(two_wifi, 8, 12);
  delay(200);
  matrix.renderBitmap(three_wifi, 8, 12);
  delay(200);
  // while (WiFi.beginAP(ssid, pass) != WL_CONNECTED) {
  //   delay(1000);
  // }
  
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

  attachInterrupt(digitalPinToInterrupt(L_HALL), leftInterrupt, FALLING);  // Trigger on falling edge
  attachInterrupt(digitalPinToInterrupt(R_HALL), rightInterrupt, FALLING);

}

void leftInterrupt() {
  Lcount++;  // Increment count when magnet passes
}

void rightInterrupt() {
  Rcount++; // Increment count when magnet passes
}

void loop() {
  //--- Wheel Speeds---//
  int speedL= 120;
  int speedR= 100;
  int turnL = 140;
  int turnR = 120;

  //--- Start/Stop boolean---//
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

          char c = client.read();

          if (c != -1){
            Serial.println(c);
          }

          // Quit / Disconnect
          if(c == 'q'){
            client.stop(); // Close the connection
            Serial.println("Client disconnected");
            running = false;
          }

          // GO!!
          if (c == 'g'){
            running = true;
            Forward(speedL, speedR);
          }
          // STOP!!
          if (c == 's'){
            running = false;
          }
        }

        //--- Line Following Code---//
        if (running) {

          

          //take in what the IR Sensor is giving us
          int current_left = digitalRead(L_EYE);
          int current_right = digitalRead(R_EYE);
          
          //Serial.println(state);

          // Stopping distance (cm)
          int stopping_dist = 20;

          if (US_ticker >= 30){
            // Serial.print("Lcount");
            // Serial.println(Lcount);
            // Serial.print("Rcount");
            // Serial.println(Rcount);
            // Serial.println( calculateDistanceTravelled());
            float distance = distance = getFilteredDistance();
            //Serial.println("Outside While loop: " + String(distance));

            // Sends US sensor data to processing
            client.println("US");
            client.println(distance);

            while (distance < stopping_dist){
              distance = getFilteredDistance();
              //Serial.println(distance);
              client.println("US");
              client.println(distance);
              Stop();
              delay(600);
              if (distance > stopping_dist){
                current_left = digitalRead(L_EYE);
                current_right = digitalRead(R_EYE);
                Forward(speedL, speedR);
                break;
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
              Left( turnR);
              current_left = digitalRead(L_EYE);
              current_right = digitalRead(R_EYE);
            }
              
            if (current_left == HIGH && current_right == LOW ) { 
            Right(turnL); 
            current_left = digitalRead(L_EYE);
            current_right = digitalRead(R_EYE);
            }

            if (current_left == LOW && current_right == LOW ) {
            Forward(speedL, speedR);
            }
          


          //else Serial.println("No Change");

          // Update the last sensor states for next check
          prev_left = current_left;
          prev_right = current_right;


          US_ticker += 1;
          delay(10);  //wait a second
          
          if (HALL_ticker >= 15){

            long currentTime = millis();
            float elapsedTime = (currentTime - prevTime) / 1000.0; // Convert to seconds

            // Sends distance travelled data to processing
            client.println("HALL");
            //client.println(calculateCurrentLeftSpeed(elapsedTime));

            float leftSpeed = calculateCurrentLeftSpeed(elapsedTime);
            float rightSpeed = calculateCurrentRightSpeed(elapsedTime);

            // Serial.print("Left Speed: ");
            // Serial.print(leftSpeed);
            // Serial.print(" - Right Speed: ");
            // Serial.println(rightSpeed);

            // Serial.print("Left PID: ");
            // Serial.print(computePID(leftSpeed, elapsedTime));
            // Serial.print(" - Right PID: ");
            // Serial.println(computePID(rightSpeed, elapsedTime));


            // Print speeds for Arduino Serial Plotter
            Serial.print(leftSpeed);
            Serial.print(",");
            Serial.println(rightSpeed); // Newline tells Serial Plotter to plot next point

            speedL = computePIDL(leftSpeed, elapsedTime);
            if (speedL > 255) speedL = 255;
            if (speedL < 0) speedL = 0;
            speedR = computePIDR(rightSpeed, elapsedTime);
            if (speedR > 255) speedR = 255;
            if (speedR < 0) speedR = 0;

            HALL_ticker = 0;

            prevTime = currentTime; // Update time reference
          }
          HALL_ticker += 1;
        }

        if (!running){
          
          Stop();
        }
      }
  }
}