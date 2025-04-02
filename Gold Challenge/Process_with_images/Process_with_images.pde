import processing.net.*;
import controlP5.*;
import meter.*;  // Import the Meter library

ControlP5 cp5;
Client myClient;

// Gauge Cluster Init //

Meter LSpeed_G, RSpeed_G, LPower_G, RPower_G;

int meter_val = 50;
int meter_scale = 250;
int gauge_cluster_origin_x = 600;
int gauge_cluster_origin_y = 75;

int leftSpeed, rightSpeed, leftPower, rightPower, goalSpeed;

String lastDistance = "--";  // Store last received value
String distanceTravelled = "--";

int maxspeed = 25;
int minspeed = 0;

int current_speed;

// Images //
PImage mario;
PImage luigi;
PImage leftimage;
PImage rightimage;
PImage demoImage;  // This is a demo image; load your image as needed

// Coordinates for the mini screen (adjust as required)
int miniX = 50;
int miniY = 275;
int miniW = 320;
int miniH = 240;

boolean vis = false;

// Example coordinates within the mini screen where an image will be drawn
int imgX = 50;
int imgY = 50;
int imgW = 80;
int imgH = 50;


// SET UP//


void setup() {
  size(1200,600);
  
  // Arduino's IP and Port
  myClient = new Client(this,"192.168.4.1",5180);
  myClient.write("I am a new client");
  cp5 = new ControlP5(this);
  
  demoImage = loadImage("fifteen_speed.png");  // Load the image
  // Optionally, load an image (ensure the file exists in your sketch's data folder)
  // demoImage = loadImage("yourImage.png");
  
  mario = loadImage("mario.jpg");
  luigi = loadImage("luigi.jpeg");
  leftimage = loadImage("left_img.png");
  rightimage = loadImage("right_img.png");
  
  cp5 = new ControlP5(this);
  
 ////
  // Gauge Cluster Setup
  ////
  
  String[] speedScaleLabels = {"0", "5", "10", "15", "20", "25", "30"};
  
  // Create a LEFT SPEED Gauge
  LSpeed_G = new Meter(this, gauge_cluster_origin_x, gauge_cluster_origin_y);
  LSpeed_G.setMeterWidth(meter_scale);
  LSpeed_G.setScaleLabels(speedScaleLabels);
  
  LSpeed_G.setMinScaleValue(0.0);
  LSpeed_G.setMaxScaleValue(30);
  
  LSpeed_G.setMinInputSignal(0);
  LSpeed_G.setMaxInputSignal(30);
  
  LSpeed_G.setTitle("Left Wheel Speed (cm/s)");
  
  // Create a RIGHT SPEED Gauge
  RSpeed_G = new Meter(this, gauge_cluster_origin_x + meter_scale, gauge_cluster_origin_y);
  RSpeed_G.setMeterWidth(meter_scale);
  RSpeed_G.setScaleLabels(speedScaleLabels);
  
  RSpeed_G.setMinScaleValue(0.0);
  RSpeed_G.setMaxScaleValue(30);
  
  RSpeed_G.setMinInputSignal(0);
  RSpeed_G.setMaxInputSignal(30);
  
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
  

  cp5.addButton("GO").setValue(0).setPosition(50,150).setSize(150,100)
  .setColorLabel(color(0, 0, 0)).setColorBackground(color(50, 250, 50));
  
  cp5.addButton("STOP").setValue(0).setPosition(300,150).setSize(150,100)
  .setColorLabel(color(0, 0, 0)).setColorBackground(color(250, 50, 50));
  //cp5.addSlider("Speed").setPosition(1000, 400).setSize(50, 150).setRange(80, 150).setValue(125)
  //.setColorLabel(color(0, 0, 0));
 

}
void draw() {
  
background(173, 216, 230);  // Clear screen every frame
  
fill(255, 255, 255);
rect(-1, -1, 550, 130); //behind title text

fill(0, 0, 0);
rect(45, 145, 150, 100); //behind button go

fill(0, 0, 0);
rect(295, 145, 150, 100); //behind button stop





fill(0, 0, 0);
rect(600, 415, 150, 150);


  image(mario, 0, 0, mario.width/6, mario.height/6);
  image(luigi, 350, 0, mario.width/4, mario.height/6); //both mario so they're the same height
  
  
  image(leftimage, 605, 420, leftimage.width/1.5, leftimage.height/1.5);
  textSize(35);
    fill(0, 0, 0);
    text("Turn Left Ahead", 800, 500); 
  //image(rightimage, 200, 300, leftimage.width, leftimage.height);
  /*
  if (present ==1) {
    image(leftimage, 605, 420, leftimage.width/1.5, leftimage.height/1.5);
    textSize(35);
    fill(0, 0, 0);
    text("Turn Left Ahead", 800, 500); 
  }
  else if (present ==2){
    image(rightimage, 605, 420, leftimage.width/1.5, leftimage.height/1.5);
    textSize(35);
    fill(0, 0, 0);
    text("Turn Left Ahead", 800, 500); 
  }
  else if (present ==3){
  }
  else if (present ==4){
  }
  
  */
  
  
    //fill(0, 0, 0);
  textSize(45);
  fill(0, 0, 0);
  text("X14 Buggy", 150, 50); 
  
    textSize(35);
  fill(187, 165, 61);
  text("Gold Edition", 150, 100); 
  
  textSize(35);
  fill(0, 0, 0);
  text("Speed & Power Gauges", 700, 50);
  
  textSize(15);
  fill(0, 0, 0);
  text("camera representation here", 100, 500);
  
  if (myClient.active()) {
    String input_string = clean_reading();  // Read once per frame
    
    if (input_string != null && input_string.equals("tag_packet")) {
        String full_data = clean_reading();  // Read the next packet (ensure it's complete)
        print(full_data);
        
        if (full_data != null && full_data.length() > 0) {  // Extra safety check
            String[] tags = split(full_data, '|');  // Split into multiple tags
            
            vis = false;  // Reset visibility flag
    
            for (String tag : tags) {
                String[] values = split(tag, ',');  // Split individual tag data
                
                if (values.length == 5) {  
                    int tagID = int(values[0]);
                    int x = int(values[1]);
                    int y = int(values[2]);
                    int w = int(values[3]);
                    int h = int(values[4]);
    
                    imgX = constrain(x - (w / 2), 0, miniW - w);
                    imgY = constrain(y - (h / 2), 0, miniH - h);
                    imgW = w;
                    imgH = h;
                    
                    if (tagID == 1) vis = true;  // Only update visibility here
                }
            }
        }
    }
  }
  
  // Update Gauge Cluster Values
  LSpeed_G.updateMeter(leftSpeed);
  RSpeed_G.updateMeter(rightSpeed);
  LPower_G.updateMeter(leftPower);
  RPower_G.updateMeter(rightPower);
  
   // Draw the mini screen border
  stroke(255);     // White border
  noFill();
  rect(miniX, miniY, miniW, miniH);

 // Example: display an image at coordinates (imgX, imgY) within the mini screen
  if (demoImage != null && vis) {
    displayImageAt(imgX, imgY, demoImage, imgW, imgH);
  }

  
  // Label the mini screen
  fill(255);
  textSize(16);
  text("HUSKY Lens View", miniX + 5, miniY + 15);
}


// This function draws an image relative to the mini screen's coordinate system.
void displayImageAt(int x, int y, PImage img, int h, int w) {
  image(img, miniX + x, miniY + y, h, w);
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


char mapDigitToChar(char digitChar) {
  int digit = digitChar - '0';  // Convert char digit to int
  return (char)('a' + digit);
}


public void GO(int theValue){
  if (myClient.active()){
    print("go");
    myClient.write("g\n");
  }
}

public void STOP(int theValue){
  if (myClient.active()){
    myClient.write("s\n");
  }
}
