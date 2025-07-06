#include <Wire.h>
#include <MPU9250_asukiaaa.h>

MPU9250_asukiaaa mySensor;

float pitch = 0.0, roll = 0.0;
unsigned long lastTime;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  mySensor.setWire(&Wire);
  mySensor.beginGyro();

  lastTime = millis();
  Serial.println("Starting gyro-only tilt estimation...");
}

void loop() {
  mySensor.gyroUpdate();

  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;  // in seconds
  lastTime = currentTime;

  float gx = mySensor.gyroX();  // Pitch rate
  float gy = mySensor.gyroY();  // Roll rate

  pitch += gx * dt;
  roll  += gy * dt;

  Serial.print("Pitch (gyro): ");
  Serial.print(pitch);
  Serial.print(" | Roll (gyro): ");
  Serial.println(roll);

  delay(10);  // Keep sample rate stable (~100 Hz)
}
