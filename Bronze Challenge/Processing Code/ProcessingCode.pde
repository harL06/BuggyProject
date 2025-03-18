import processing.net.*;
import controlP5.*;
import meter.*;  // Import the Meter library

// Gauge Cluster Init //

Meter LSpeed_G;
Meter RSpeed_G;
Meter LPower_G;
Meter RPower_G;

int meter_val = 50;

int meter_scale = 250;
int gauge_cluster_origin_x = 400;
int gauge_cluster_origin_y = 0;

int leftSpeed;
int rightSpeed;
int leftPower;
int rightPower;

// ----

ControlP5 cp5;
Client myClient;

String lastDistance = "--";  // Store last received value
String distanceTravelled = "--";


void setup() {
  cp5 = new ControlP5(this);

  size(1200,600);
  
  ////
  // Gauge Cluster Setup
  ////
  
  String[] speedScaleLabels = {"0", "10", "20", "30", "40", "50", "60"};
  
  // Create a LEFT SPEED Gauge
  LSpeed_G = new Meter(this, gauge_cluster_origin_x, gauge_cluster_origin_y);
  LSpeed_G.setMeterWidth(meter_scale);
  LSpeed_G.setScaleLabels(speedScaleLabels);
  
  LSpeed_G.setMinScaleValue(0.0);
  LSpeed_G.setMaxScaleValue(60.0);
  
  LSpeed_G.setMinInputSignal(0);
  LSpeed_G.setMaxInputSignal(60);
  
  LSpeed_G.setTitle("Left Wheel Speed (cm/s)");
  
  // Create a RIGHT SPEED Gauge
  RSpeed_G = new Meter(this, gauge_cluster_origin_x + meter_scale, gauge_cluster_origin_y);
  RSpeed_G.setMeterWidth(meter_scale);
  RSpeed_G.setScaleLabels(speedScaleLabels);
  
  RSpeed_G.setMinScaleValue(0.0);
  RSpeed_G.setMaxScaleValue(60.0);
  
  RSpeed_G.setMinInputSignal(0);
  RSpeed_G.setMaxInputSignal(60);
  
  RSpeed_G.setTitle("Right Wheel Speed (cm/s)");
  
  String[] powerScaleLabels = {"0", "25", "50", "75", "100", "125", "150", "175", "200", "225", "250", "275"};
  
  // Create a LEFT Power Gauge
  LPower_G = new Meter(this, gauge_cluster_origin_x, gauge_cluster_origin_y + floor(meter_scale/1.7));
  LPower_G.setMeterWidth(meter_scale);
  LPower_G.setScaleLabels(powerScaleLabels);
  
  LPower_G.setMinScaleValue(0.0);
  LPower_G.setMaxScaleValue(275.0);
  
  LPower_G.setMinInputSignal(0);
  LPower_G.setMaxInputSignal(275);
  
  LPower_G.setTitle("Left Wheel Power (digital)");
  
  // Create a Right Power Gauge
  RPower_G = new Meter(this, gauge_cluster_origin_x + meter_scale, gauge_cluster_origin_y + floor(meter_scale/1.7));
  RPower_G.setMeterWidth(meter_scale);
  RPower_G.setScaleLabels(powerScaleLabels);
  
  RPower_G.setMinScaleValue(0.0);
  RPower_G.setMaxScaleValue(275.0);
  
  RPower_G.setMinInputSignal(0);
  RPower_G.setMaxInputSignal(275);
  
  RPower_G.setTitle("Right Wheel Power (digital)");

  ////
  // ----
  ////
  
  cp5.addButton("Disconnect").setValue(0).setPosition(100,70).setSize(200,19);
  cp5.addButton("GO").setValue(0).setPosition(100,100).setSize(200,19);
  cp5.addButton("STOP").setValue(0).setPosition(100,130).setSize(200,19);
  cp5.addLabel("US_Sensor").setText("").setPosition(0,0);
  
  // Arduino's IP and Port
  myClient = new Client(this,"192.168.4.1",5180);
  //myClient.write("I am a new client");
}
void draw() {
  
  background(51, 51, 51);  // Clear screen every frame

if (myClient.active()) {
    String input_string = clean_reading();  // Read once

    // Check if the input string is not empty
    if (input_string != "") {
        // Print input for debugging purposes (optional)
        // print("INPUT: " + input_string + "\n");

        // If "US" is received, get the next value (distance)
        if (input_string.equals("US")) {
            lastDistance = clean_reading();  // Read the next line (distance)
            //print("Last Distance: " + lastDistance + "\n");
        }
        
        // If "L_speed" is received, get the next value (left wheel speed)
        if (input_string.equals("speed_packet")) {
            input_string = clean_reading();
            
            String[] values = split(input_string, ',');  // Split by comma
    
            if (values.length == 4) {  // Ensure we got the expected number of values
                leftSpeed  = int(values[0]);
                rightSpeed = int(values[1]);
                leftPower  = int(values[2]);
                rightPower = int(values[3]);
    
                println("Left Speed: " + leftSpeed + " Right Speed: " + rightSpeed + " Left Power: " + leftPower + " Right Power: " + rightPower);
            }
        }
    }
}
  
  fill(255);
  textSize(14);
  // Display last valid distance even if no new data
  text("US Sensor Reading: " + lastDistance + " cm", 10, 30);
  //m.updateMeter(int(lastDistance));
  
  //text("Distance Travelled: " + distanceTravelled + " cm", 10, 50);
  
  if (float(lastDistance) < 20) {
    fill(255, 0, 0);
    textSize(14);
    text("Buggy Stopping!", 200, 30);
  }
  
  // Update Gauge Cluster Values
  LSpeed_G.updateMeter(leftSpeed);
  RSpeed_G.updateMeter(rightSpeed);
  LPower_G.updateMeter(leftPower);
  RPower_G.updateMeter(rightPower);

}

public String clean_reading(){
    if (myClient != null && myClient.available() > 0) {  // Ensure client is valid
      String reading = myClient.readStringUntil('\n');
  
      if (reading != null && !reading.isEmpty()) {
        reading = reading.replaceAll("[\\r\\n]+", "").trim();
        //print(reading + "\n");
         return reading;
      }
    }
    return "";
}


public void Disconnect(int theValue){
  if (myClient.active()){
    myClient.write("q");
  }
}

public void GO(int theValue){
  if (myClient.active()){
    myClient.write("g");
  }
}

public void STOP(int theValue){
  if (myClient.active()){
    myClient.write("s");
  }
}
