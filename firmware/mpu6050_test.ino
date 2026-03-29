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

  // Convert to g-forces and degrees/second (example scaling)
  float accel_x = ax / 16384.0;  // ±2g range
  float accel_y = ay / 16384.0;
  float accel_z = az / 16384.0;
  float gyro_x = gx / 131.0;     // ±250°/s range
  float gyro_y = gy / 131.0;
  float gyro_z = gz / 131.0;

  Serial.print("ax: "); Serial.print(accel_x, 2);
  Serial.print(" ay: "); Serial.print(accel_y, 2);
  Serial.print(" az: "); Serial.print(accel_z, 2);
  Serial.print(" gx: "); Serial.print(gyro_x, 2);
  Serial.print(" gy: "); Serial.print(gyro_y, 2);
  Serial.print(" gz: "); Serial.println(gyro_z, 2);

  delay(20); // ~50Hz
}
