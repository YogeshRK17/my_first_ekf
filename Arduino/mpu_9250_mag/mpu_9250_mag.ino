#include <Wire.h>
#include <MPU9250_asukiaaa.h>

MPU9250_asukiaaa mySensor;


void setup() {
  Serial.begin(115200);
  Wire.begin();

  mySensor.setWire(&Wire);
  mySensor.beginMag();
}

void loop(){
  mySensor.magUpdate();
  mySensor.accelUpdate();
  
  float mx = mySensor.magX();
  float my = mySensor.magY();
  float mz = mySensor.magZ();
  Serial.print(mx);
  Serial.print(my);
  Serial.print(mz);

  float ax = mySensor.accelX();
  float ay = mySensor.accelY();
  float az = mySensor.accelZ();

  float pitch = -atan2(ax, sqrt(ay*ay + az*az));
  float roll  = atan2(ay, sqrt(ax*ax + az*az));

  // Step 2: Tilt-compensated magnetometer
  float Xh = mx * cos(pitch) + mz * sin(pitch);
  float Yh = mx * sin(roll) * sin(pitch) + my * cos(roll) - mz * sin(roll) * cos(pitch);
  float yaw = atan2(-Yh, Xh);

  // Convert to degrees
  yaw = yaw * 180 / PI;
  
  if (yaw < 0) yaw += 360;
  Serial.println(yaw);

  delay(20);
}
