#include <Wire.h>
#include <MPU9250_asukiaaa.h>

MPU9250_asukiaaa mySensor;

float pitch = 0.0, roll = 0.0, yaw = 0.0;
unsigned long lastTime;
const float alpha = 0.98;
float pitch_f = 0.0, roll_f = 0.0, yaw_f = 0.0;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  mySensor.setWire(&Wire);
  mySensor.beginAccel();
  mySensor.beginGyro();

  lastTime = millis();
  Serial.println("MPU9250 initialized, for tilt estimation");
}

void loop() {
//  get_accelero_data();
//  get_gyro_data();
//  get_both_pitch_roll();
  get_both_pitch_roll_filtered();
  delay(50);
}
