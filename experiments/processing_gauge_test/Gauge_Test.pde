import meter.*;  // Import the Meter library

Meter LSpeed_G;
Meter RSpeed_G;
Meter LPower_G;
Meter RPower_G;

int meter_val = 50;

int meter_scale = 250;
int gauge_cluster_origin_x = 0;
int gauge_cluster_origin_y = 0;

void setup() {
  size(1200,600);
  
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
  
  String[] powerScaleLabels = {"0", "25", "50", "75", "100", "125", "150", "175", "200", "225", "250"};
  
  // Create a LEFT Power Gauge
  LPower_G = new Meter(this, gauge_cluster_origin_x, gauge_cluster_origin_y + floor(meter_scale/1.7));
  LPower_G.setMeterWidth(meter_scale);
  LPower_G.setScaleLabels(powerScaleLabels);
  
  LPower_G.setMinScaleValue(0.0);
  LPower_G.setMaxScaleValue(255.0);
  
  LPower_G.setMinInputSignal(0);
  LPower_G.setMaxInputSignal(255);
  
  LPower_G.setTitle("Left Wheel Power (digital)");
  
  // Create a Right Power Gauge
  RPower_G = new Meter(this, gauge_cluster_origin_x + meter_scale, gauge_cluster_origin_y + floor(meter_scale/1.7));
  RPower_G.setMeterWidth(meter_scale);
  RPower_G.setScaleLabels(powerScaleLabels);
  
  RPower_G.setMinScaleValue(0.0);
  RPower_G.setMaxScaleValue(255.0);
  
  RPower_G.setMinInputSignal(0);
  RPower_G.setMaxInputSignal(255);
  
  RPower_G.setTitle("Right Wheel Power (digital)");
  
}

void draw() {
  
  background(51, 51, 51);  // Clear screen every frame
  LSpeed_G.updateMeter(meter_val);
  RSpeed_G.updateMeter(meter_val);
  LPower_G.updateMeter(meter_val);
  RPower_G.updateMeter(meter_val);
}
