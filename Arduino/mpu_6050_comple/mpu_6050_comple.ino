#include <Wire.h>
#include <MPU9250_asukiaaa.h>

MPU9250_asukiaaa mySensor;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  mySensor.setWire(&Wire);
  mySensor.beginAccel();
  mySensor.beginGyro();
  mySensor.beginMag(); // optional

  Serial.println("MPU9250 initialized");
}

void loop() {
  mySensor.accelUpdate();
  mySensor.gyroUpdate();

  float ax = mySensor.accelX();
  float ay = mySensor.accelY();
  float az = mySensor.accelZ();

//  // Calculate pitch and roll using accelerometer
//  float pitch = atan2(ax, sqrt(ay * ay + az * az)) * 180 / PI;
//  float roll = atan2(ay, sqrt(ax * ax + az * az)) * 180 / PI;
//
//  Serial.print("Pitch: ");
//  Serial.print(pitch);
//  Serial.print(" | Roll: ");
//  Serial.println(roll);
    Serial.print("ax: ");
    Serial.println(ax);
    Serial.print("ay: ");
    Serial.println(ay);
    Serial.print("az: ");
    Serial.println(az);
  delay(100);
}
