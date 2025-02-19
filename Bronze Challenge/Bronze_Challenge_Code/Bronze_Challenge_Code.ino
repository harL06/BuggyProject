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

#define L_MOTOR_IN1 5   //left motor pin 1
#define L_MOTOR_IN2 6   //left motor pin 2
#define R_MOTOR_IN1 12  //right motor pin 1
#define R_MOTOR_IN2 4  //right motor pin 2

#define TRIG_PIN 3  // Trig pin connected to D2
#define ECHO_PIN 2 // Echo pin connected to D4

#define L_HALL 0  // Trig pin connected to D2
#define R_HALL 1 // Echo pin connected to D4


volatile int Lcount = 0; // Counts how many times hall sensor picks up left/right wheel
volatile int Rcount = 0;


const float WHEEL_RAD = 3; //cm
const float WHEEL_CIRCUM =  PI * WHEEL_RAD * 2;


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

    if (dist > 400) {  // Discard absurd readings (sensor max is ~400 cm)
      return -1;  // Return an invalid value
  }

  return dist;
}


float getFilteredDistance() {
    float sum = 0;
    int samples = 5;  // Number of readings to average
    float pulse = 0;
    for (int i = 0; i < samples; i++) {
        pulse = US_Pulse();
        if (pulse != -1) sum += US_Pulse();
        delay(10);  // Small delay between readings
    }
    return sum / samples;  // Return the average distance
}


float calculateDistanceTravelled(){
  float mean_count = (float(Lcount) + float(Rcount))/2;
  Serial.print("mean_count: ");
  Serial.println(mean_count);
  float wheel_rotations = mean_count / 4; // 4 counts for every 1 wheel rotation
  Serial.print("wheel_rotations: ");
  Serial.println(wheel_rotations);
  return (WHEEL_CIRCUM * wheel_rotations);
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
  int speedL= 115;
  int speedR= 90;
  int turnL = 120;
  int turnR = 120;

  //--- Start/Stop boolean---//
  bool running = false;
  float US_dist;
  int US_ticker = 0;
  
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

          if (US_ticker >= 20){
            Serial.print("Lcount");
            Serial.println(Lcount);
            Serial.print("Rcount");
            Serial.println(Rcount);
            Serial.println( calculateDistanceTravelled());
            float distance = distance = getFilteredDistance();
            //Serial.println("Outside While loop: " + String(distance));
            client.println(distance);
            while (distance < stopping_dist){
              distance = getFilteredDistance();
              //Serial.println(distance);
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
        }

        if (!running){
          
          Stop();
        }
      }
  }
}