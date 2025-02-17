import processing.net.*;
import controlP5.*;


ControlP5 cp5;
Client myClient;

String lastDistance = "--";  // Store last received value

void setup() {
  cp5 = new ControlP5(this);

  size(400,200);
  
  
  
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
    String reading = myClient.readStringUntil('\n');
    
    if (reading != null) {
      reading = reading.replaceAll("[\\r\\n]+", "").trim();
      
      if (!reading.isEmpty()) {  
        lastDistance = reading;  // Store latest valid reading
        //println(lastDistance);
      }
    }
  }
  fill(255);
  textSize(14);
  // Display last valid distance even if no new data
  text("US Sensor Reading: " + lastDistance + "cm", 10, 30);
  
  if (float(lastDistance) < 20) {
    fill(255, 0, 0);
    textSize(14);
    text("Buggy Stopping!", 10, 50);
  }

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
