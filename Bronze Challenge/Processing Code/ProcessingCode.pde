import processing.net.*;
import controlP5.*;
import meter.*;  // Import the Meter library




ControlP5 cp5;
Client myClient;

Meter m;

String lastDistance = "--";  // Store last received value
String distanceTravelled = "--";

void setup() {
  cp5 = new ControlP5(this);

  size(400,300);
  
  // Create a circular meter (gauge)
  m = new Meter(this, 400, 0);
  String[] scaleLabels = {"0", "10", "20", "30"};
  m.setScaleLabels(scaleLabels);
  m.setMaxScaleValue(60);
  m.setTitle("US Distance");
  
  
  
  cp5.addButton("Disconnect").setValue(0).setPosition(100,70).setSize(200,19);
  cp5.addButton("GO").setValue(0).setPosition(100,100).setSize(200,19);
  cp5.addButton("STOP").setValue(0).setPosition(100,130).setSize(200,19);
  cp5.addLabel("US_Sensor").setText("").setPosition(0,0);
  
  // Arduino's IP and Port
  myClient = new Client(this,"192.168.4.1",5180);
  //myClient.write("I am a new client");
}
void draw() {
  
  background(153, 196, 210);  // Clear screen every frame

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
        
        // If "HALL" is received, get the next value (distance travelled)
        if (input_string.equals("HALL")) {
            distanceTravelled = clean_reading();  // Read the next line (distance travelled)
            //print("Distance Travelled: " + distanceTravelled + "\n");
        }
    }
}
  
  fill(255);
  textSize(14);
  // Display last valid distance even if no new data
  text("US Sensor Reading: " + lastDistance + " cm", 10, 30);
  m.updateMeter(int(lastDistance));
  
  text("Distance Travelled: " + distanceTravelled + " cm", 10, 50);
  
  if (float(lastDistance) < 20) {
    fill(255, 0, 0);
    textSize(14);
    text("Buggy Stopping!", 200, 30);
  }

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
