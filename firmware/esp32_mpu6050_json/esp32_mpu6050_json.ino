/**
 * RoboSim ESP32 + MPU6050 Firmware
 * 
 * Outputs JSON strings over Serial for real-time robot control.
 * Format: {"t": ms, "r": roll, "p": pitch, "y": yaw, "gx": gx, "gy": gy, "gz": gz}
 * 
 * Dependencies:
 * - Install "MPU6050_tockn" or "MPU6050_light" via Arduino Library Manager
 */

#include <Wire.h>
#include <MPU6050_light.h> // Highly recommended for easy roll/pitch/yaw

MPU6050 mpu(Wire);
unsigned long timer = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(); // SDA=21, SCL=22 on most ESP32s
  
  byte status = mpu.begin();
  if (status != 0) {
    Serial.print("{\"error\": \"MPU6050 init failed with status ");
    Serial.print(status);
    Serial.println("\"}");
    while (1) {}
  }
  
  // Calibration phase (keep sensor still)
  // The app will see zeros during this time or we can just wait
  mpu.calcOffsets(); 
}

void loop() {
  mpu.update();

  if ((millis() - timer) > 20) { // 50Hz update rate
    Serial.print("{");
    Serial.print("\"t\":");   Serial.print(millis());
    Serial.print(",\"r\":");  Serial.print(mpu.getAngleX(), 2);
    Serial.print(",\"p\":");  Serial.print(mpu.getAngleY(), 2);
    Serial.print(",\"y\":");  Serial.print(mpu.getAngleZ(), 2);
    Serial.print(",\"gx\":"); Serial.print(mpu.getGyroX(), 2);
    Serial.print(",\"gy\":"); Serial.print(mpu.getGyroY(), 2);
    Serial.print(",\"gz\":"); Serial.print(mpu.getGyroZ(), 2);
    Serial.println("}");
    
    timer = millis();
  }
}
