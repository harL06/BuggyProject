import controlP5.*;          //imported library
import processing.serial.*;  //existing library
Serial port;                 //sets port
ControlP5 cp5;               //creates object
PFont font;                  //set font

void setup(){
size(800, 500);
//port = new Serial(this, "COM6" <- this is not wifi)
//setup arduino connection - check how to do this

//aesthethics - TBD
font = createFont("Georgia" , 20);   //picked a font
textFont(font);
fill(0, 0, 0); //text black
text("X14 - Buggy Controls", 10, 30);
line(0, 50, 210, 50);
line(210, 50, 210, 0);

//Button (theoretically this should work)
cp5.addButton("GO")
  .setPosition(50, 100)
  .setSize(150, 150)
  .setFont(font)
  .setColorBackground(color(165, 242, 141))
  .setColorForeground(color(153, 204, 255))
  .setColorLabel(color(0,0,0))
  ;
}

void draw(){
  background(173, 216, 230);   //pale blue
}

void GO(){
  port.write(1);
}
