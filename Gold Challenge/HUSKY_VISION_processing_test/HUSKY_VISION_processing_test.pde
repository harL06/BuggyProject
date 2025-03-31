PImage demoImage;  // This is a demo image; load your image as needed
import processing.net.*;

Client myClient;

// Coordinates for the mini screen (adjust as required)
int miniX = 50;
int miniY = 50;
int miniW = 320;
int miniH = 240;

// Example coordinates within the mini screen where an image will be drawn
int imgX = 50;
int imgY = 50;
int imgW = 80;
int imgH = 50;

boolean vis = false;

void setup() {
  
   myClient = new Client(this,"192.168.4.1",5180);
   

   
   
  size(640, 480);  // Set the overall window size (modify as needed)
  demoImage = loadImage("fifteen_speed.png");  // Load the image
  // Optionally, load an image (ensure the file exists in your sketch's data folder)
  // demoImage = loadImage("yourImage.png");
}

void draw() {
  background(0);  // Clear the background
  
  if (myClient.active()){
    myClient.write("g\n");
  }
  
  if (myClient.active()) {
    String input_string = clean_reading();  // Read once per frame
    
    if (input_string != null && input_string.equals("tag_packet")) {
        String full_data = clean_reading();  // Read the next packet (ensure it's complete)
        
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
