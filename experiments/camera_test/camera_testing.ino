#include "HUSKYLENS.h"

HUSKYLENS huskylens;

void setup() {
  Serial.begin(115200);

  Wire.begin();
  // Connect I2C bus and huskylens.
  // Returns false if there is a problem, so I check for that 
  // and print errors until things are working.
  while( !huskylens.begin(Wire) ){
    Serial.println( F("Huskylens begin failed!") );
    Serial.println( F("Check Huskylens protocol is set to I2C (General > Settings > Protocol Type > I2C") );
    Serial.println( F("And confirm the physical connection."));
    delay(1000); // Wait a second before trying to initialise again.
  }
}

void loop() {
  
  // First, check that we have the huskylens connected...
  if (!huskylens.request()) Serial.println(F("Fail to request data from HUSKYLENS, recheck the connection!"));
  // then check that it's been trained on something...
  else if (!huskylens.isLearned()) Serial.println(F("Nothing learned, press learn button on HUSKYLENS to learn one!"));
  // Then check whether there are any blocks visible at this exact moment...
  //else if (!huskylens.available()) Serial.println(F("No block or arrow appears on the screen!"));
  else {
    // OK, we have some blocks to process. available() will return the number of blocks to work through.
    // fetch them using read(), one at a time, until there are none left. Each block gets given to
    // my printResult() function to be printed out to the serial port.
    int closestID = 0;
    float maxIdSize = 0;
    while (huskylens.available()) {
      HUSKYLENSResult result = huskylens.read();
      //printResult(result);
      if ((result.width * result.height) > maxIdSize){
        maxIdSize = result.width * result.height;
        closestID = result.ID;
      }
    }
    if (closestID != 0){
      Serial.println(closestID);
    }
  }

}

// Take a huskylens result and print out the x/y coordinates and other useful properties.
void printResult(HUSKYLENSResult result) {
  if (result.command == COMMAND_RETURN_BLOCK) {
    Serial.println(String()+F("Block:xCenter=")+result.xCenter+F(",yCenter=")+result.yCenter+F(",width=")+result.width+F(",height=")+result.height+F(",ID=")+result.ID);
  } else if (result.command == COMMAND_RETURN_ARROW) {
    Serial.println(String()+F("Arrow:xOrigin=")+result.xOrigin+F(",yOrigin=")+result.yOrigin+F(",xTarget=")+result.xTarget+F(",yTarget=")+result.yTarget+F(",ID=")+result.ID);
  } else {
    Serial.println("Object unknown!");
  }
}