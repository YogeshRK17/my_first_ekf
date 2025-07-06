#include <Wire.h>
#include <MPU9250_asukiaaa.h>

MPU9250_asukiaaa mySensor;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  mySensor.setWire(&Wire);
  mySensor.beginAccel();

  Serial.println("MPU9250 initialized, for tilt estimation");
}

void loop() {
  mySensor.accelUpdate();

  float ax = mySensor.accelX();
  float ay = mySensor.accelY();
  float az = mySensor.accelZ();

  // Calculate pitch and roll using accelerometer
  float pitch = atan2(ax, sqrt(ay * ay + az * az)) * 180 / PI;
  float roll = atan2(ay, sqrt(ax * ax + az * az)) * 180 / PI;

  Serial.print("Pitch: ");
  Serial.print(pitch);
  Serial.print(" | Roll: ");
  Serial.println(roll);
  delay(100);
}
