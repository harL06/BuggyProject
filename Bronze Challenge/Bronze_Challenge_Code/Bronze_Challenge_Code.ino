// Group X14

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
#define R_MOTOR_IN2 13  //right motor pin 2

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

//initialises a place to hold the previous IR input
int prev_left = HIGH;
int prev_right = HIGH;

void setup() {
 Serial.begin(9600);
  
  //IR Sensors send us signals, we send motors signals
  pinMode(L_EYE, INPUT);
  pinMode(R_EYE, INPUT);
  pinMode(L_MOTOR_IN1, OUTPUT);
  pinMode(L_MOTOR_IN2, OUTPUT);
  pinMode(R_MOTOR_IN1, OUTPUT);
  pinMode(R_MOTOR_IN2, OUTPUT);

}

void loop() {

//--- Wheel Speeds---//

  int speedL= 115;
  int speedR= 90;
  int turnL = 120;
  int turnR = 120;

//--- Sensor Readings---//

  //take in what the IR Sensor is giving us
  int current_left = digitalRead(L_EYE);
  int current_right = digitalRead(R_EYE);

  if (current_left != prev_left || current_right != prev_right) {
    Serial.print("Left Sensor Change: ");
    Serial.print(current_left == HIGH ? "HIGH" : "LOW");
    Serial.print("Right Sensor Change: ");
    Serial.println(current_right == HIGH ? "HIGH" : "LOW");
  } else{
    Serial.print("Left Sensor: ");
    Serial.print(current_left == HIGH ? "HIGH" : "LOW");
    Serial.print("Right Sensor: ");
    Serial.print(current_right == HIGH ? "HIGH" : "LOW");
  }
  
////--- Readings and Outputs ---//

  //call forward function
  if (current_left == HIGH && current_right == HIGH ) { 
    Forward(speedL, speedR); 
    }

  if (current_left == LOW && current_right == HIGH ) { 
    Left( turnR);
    while (current_left == LOW ){
      current_left = digitalRead(L_EYE);
    }
    current_left = digitalRead(L_EYE);
    current_right = digitalRead(R_EYE);
  }
    
  if (current_left == HIGH && current_right == LOW ) { 
    Right(turnL); 
    while (current_right == LOW){
      current_right = digitalRead(R_EYE);
    }
    current_left = digitalRead(L_EYE);
    current_right = digitalRead(R_EYE);
  }

  if (current_left == LOW && current_right == LOW ) {
    Stop();
  }

    // Update the last sensor states for next check
  prev_left = current_left;
  prev_right = current_right;
  
  delay(10);  //wait a second
}