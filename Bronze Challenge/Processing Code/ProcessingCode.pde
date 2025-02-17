import processing.net.*;
import controlP5.*;

ControlP5 cp5;
Client myClient;
String data;

void setup() {
  cp5 = new ControlP5(this);

  size(400,200);
  cp5.addButton("Disconnect").setValue(0).setPosition(100,70).setSize(200,19);
  cp5.addButton("GO").setValue(0).setPosition(100,100).setSize(200,19);
  cp5.addButton("STOP").setValue(0).setPosition(100,130).setSize(200,19);
  
  // Arduino's IP and Port
  myClient = new Client(this,"192.168.4.1",5180);
  //myClient.write("I am a new client");
}
void draw() {
  if (!myClient.active()) exit();
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
