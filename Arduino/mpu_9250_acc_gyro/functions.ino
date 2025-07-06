void get_accelero_data()
{
  mySensor.accelUpdate();

  float ax = mySensor.accelX();
  float ay = mySensor.accelY();
  float az = mySensor.accelZ();

  // Calculate pitch and roll using accelerometer
  float pitch_a = atan2(ax, sqrt(ay * ay + az * az)) * 180 / PI;
  float roll_a  = atan2(ay, sqrt(ax * ax + az * az)) * 180 / PI;

//  Serial.print("Pitch acc: ");
//  Serial.print(pitch_a);
//  Serial.print(" | Roll acc: ");
//  Serial.println(roll_a);

  Serial.print(pitch_a);
  Serial.print(",");
  Serial.println(roll_a);
}

void get_gyro_data()
{
  mySensor.gyroUpdate();

  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;  // in seconds
  lastTime = currentTime;

  float gx = mySensor.gyroX();  // Pitch rate
  float gy = mySensor.gyroY();  // Roll rate
  float gz = mySensor.gyroZ();  // Yaw rate

  roll  += gx * dt;
  pitch += gy * dt;
  yaw   += gz * dt;

//  Serial.print("Pitch (gyro): ");
//  Serial.print(pitch);
//  Serial.print(" | Roll (gyro): ");
//  Serial.println(roll);
//  Serial.print(" | Yaw (gyro): ");
//  Serial.println(yaw);

  Serial.print(pitch);
  Serial.print(",");
  Serial.println(roll);
//  Serial.print(",");
//  Serial.println(yaw);
}

void get_both_pitch_roll()
{
  // set sensor update
  mySensor.accelUpdate();
  mySensor.gyroUpdate();

  // set accelerometer
  float ax = mySensor.accelX();
  float ay = mySensor.accelY();
  float az = mySensor.accelZ();

  // set gyroscope
  float gx = mySensor.gyroX();  // Pitch rate
  float gy = mySensor.gyroY();  // Roll rate
  float gz = mySensor.gyroZ();  // Yaw rate

  // Calculate pitch and roll using accelerometer
  float pitch_a = -atan2(ax, sqrt(ay * ay + az * az)) * 180 / PI;
  float roll_a  = atan2(ay, sqrt(ax * ax + az * az)) * 180 / PI;

  // Calculate pitch and roll using gyroscope
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;  // in seconds
  lastTime = currentTime;

  roll  += gx * dt;
  pitch += gy * dt;
  yaw   += gz * dt;

  Serial.print(pitch_a);
  Serial.print(",");
  Serial.print(roll_a);
  Serial.print(",");
  Serial.print(pitch);
  Serial.print(",");
  Serial.print(roll);
  Serial.print(",");
  Serial.println(yaw);
}

void get_both_pitch_roll_filtered()
{
  // set sensor update
  mySensor.accelUpdate();
  mySensor.gyroUpdate();

  // set accelerometer
  float ax = mySensor.accelX();
  float ay = mySensor.accelY();
  float az = mySensor.accelZ();

  // set gyroscope
  float gx = mySensor.gyroX();  // Pitch rate
  float gy = mySensor.gyroY();  // Roll rate
  float gz = mySensor.gyroZ();  // Yaw rate

  // Calculate pitch and roll using accelerometer
  float pitch_a = -atan2(ax, sqrt(ay * ay + az * az)) * 180 / PI;
  float roll_a  = atan2(ay, sqrt(ax * ax + az * az)) * 180 / PI;

  // Calculate pitch and roll using gyroscope
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;  // in seconds
  lastTime = currentTime;

  pitch += gy * dt;
  roll  += gx * dt;

  pitch_f = alpha * (pitch_f + gy * dt) + (1-alpha) * pitch_a;
  roll_f  = alpha * (roll_f + gx * dt) + (1-alpha) * roll_a;
  yaw   += gz * dt;

  Serial.print(pitch_a);
  Serial.print(",");
  Serial.print(roll_a);
  Serial.print(",");
  Serial.print(pitch);
  Serial.print(",");
  Serial.print(roll);
  Serial.print(",");
  Serial.print(pitch_f);
  Serial.print(",");
  Serial.print(roll_f);
  Serial.print(",");
  Serial.println(yaw);
}
