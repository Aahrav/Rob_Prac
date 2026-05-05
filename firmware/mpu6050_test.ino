/**
 * MPU-6050 Raw Data Test
 *
 * Prints raw accelerometer and gyroscope values to Serial at 115200 baud.
 * Used for Step 1: Hardware Verification.
 */

#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

const long BAUD_RATE = 115200;

void setup() {
  Serial.begin(BAUD_RATE);
  Wire.begin(); // SDA=21, SCL=22 on ESP32 default

  Serial.println("Initializing MPU-6050...");
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU-6050 connection failed");
    while (1) {
      delay(1000);
      Serial.println("MPU-6050 connection failed - check wiring");
    }
  }

  Serial.println("MPU-6050 initialized. Reading raw values.");
}

void loop() {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Convert to g-forces and degrees/second
  float accel_x = ax / 16384.0;
  float accel_y = ay / 16384.0;
  float accel_z = az / 16384.0;
  float gyro_x  = gx / 131.0;
  float gyro_y  = gy / 131.0;
  float gyro_z  = gz / 131.0;

  // Calculate Roll and Pitch from Accelerometer (degrees)
  // atan2(ay, az) gives roll; atan2(-ax, sqrt(ay^2 + az^2)) gives pitch
  float roll  = atan2(accel_y, accel_z) * 180.0 / PI;
  float pitch = atan2(-accel_x, sqrt(accel_y * accel_y + accel_z * accel_z)) * 180.0 / PI;
  float yaw   = 0.0; // Yaw cannot be accurately determined from accel alone

  // Output JSON string
  Serial.print("{\"t\":");
  Serial.print(millis());
  Serial.print(",\"r\":");
  Serial.print(roll, 2);
  Serial.print(",\"p\":");
  Serial.print(pitch, 2);
  Serial.print(",\"y\":");
  Serial.print(yaw, 2);
  Serial.print(",\"gx\":");
  Serial.print(gyro_x, 2);
  Serial.print(",\"gy\":");
  Serial.print(gyro_y, 2);
  Serial.print(",\"gz\":");
  Serial.print(gyro_z, 2);
  Serial.println("}");

  delay(10); // ~100Hz output rate
}
