#include "Wire.h" // This library allows you to communicate with I2C devices.

const int MPU_ADDR = 0x68; // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.
const int DELAY = 20;
const int CALIBRATION_ITERS = 200;
const int PRINT_RATE = 50;

float accX, accY, accZ; // variables for accelerometer raw data
float gyroX, gyroY, gyroZ; // variables for gyro raw data
float gyroAngleX = 0, gyroAngleY = 0, gyroAngleZ = 0;
float temperature; // variable for temperature data
float elapsedTime = 0, currentTime = 0, previousTime = 0; // time deltas
float errorGyroX = 0, errorGyroY = 0, errorGyroZ = 0;
float errorAccX = 0, errorAccY = 0, errorAccZ = 0;
float roll = 0, pitch = 0, yaw = 0;
float accAngleX = 0, accAngleY = 0;
int counter = 0;
char tmp_str[7]; // temporary variable used in convert function

char* convert_int16_to_str(int16_t i) { // converts int16 to string. Moreover, resulting strings will have the same length in the debug monitor.
  sprintf(tmp_str, "%6d", i);
  return tmp_str;
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  for(int i = 0;i<10;++i) {
    Serial.println();
  }
  calculateImuError();
}

void calculateImuError() {
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
  Serial.println("====================New run====================");
  Serial.println("Calibrating initial offsets...");
  for (int i = 0; i < CALIBRATION_ITERS; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
    Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
    Wire.requestFrom(MPU_ADDR, 7*2, true);
    accX = (Wire.read() << 8 | Wire.read()) / 16384.0;
    accY = (Wire.read() << 8 | Wire.read()) / 16384.0;
    accZ = (Wire.read() << 8 | Wire.read()) / 16384.0;
    temperature = (Wire.read()<<8 | Wire.read()) / 340.0 + 36.53;
    gyroX = (Wire.read() << 8 | Wire.read()) / 131.0;
    gyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
    gyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;

    // Sum up errors
    errorAccX += ((atan((accY) / sqrt(pow((accX), 2) + pow((accZ), 2))) * 180 / PI));
    errorAccY += ((atan(-1 * (accX) / sqrt(pow((accY), 2) + pow((accZ), 2))) * 180 / PI));
    errorGyroX += gyroX;
    errorGyroY += gyroY;
    errorGyroZ += gyroZ;
    delay(DELAY);
  }
  //Divide the sum by CALIBRATION_ITERS to get the error value
  errorAccX /= CALIBRATION_ITERS;
  errorAccY /= CALIBRATION_ITERS;
  errorGyroX /= CALIBRATION_ITERS;
  errorGyroY /= CALIBRATION_ITERS;
  errorGyroZ /= CALIBRATION_ITERS;
  // Print the error values on the Serial Monitor
  Serial.print("errorAccX: ");
  Serial.println(errorAccX);
  Serial.print("errorAccY: ");
  Serial.println(errorAccY);
  Serial.print("errorAccZ: ");
  Serial.println(errorAccZ);
  Serial.print("errorGyroX: ");
  Serial.println(errorGyroX);
  Serial.print("errorGyroY: ");
  Serial.println(errorGyroY);
  Serial.print("errorGyroZ: ");
  Serial.println(errorGyroZ);
}
void getImu(){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
  Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire.requestFrom(MPU_ADDR, 7*2, true); // request a total of 7*2=14 registers
  
  // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
  accX = (Wire.read()<<8 | Wire.read()) / 16384.0; // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  accY = (Wire.read()<<8 | Wire.read()) / 16384.0; // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  accZ = (Wire.read()<<8 | Wire.read()) / 16384.0; // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
  temperature = (Wire.read()<<8 | Wire.read()) / 340.0 + 36.53; // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
  gyroX = (Wire.read()<<8 | Wire.read()) / 131.0 - errorGyroX; // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
  gyroY = (Wire.read()<<8 | Wire.read()) / 131.0 - errorGyroY; // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
  gyroZ = (Wire.read()<<8 | Wire.read()) / 131.0 - errorGyroZ; // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)

  // update tracked variables
  previousTime = currentTime;
  currentTime = millis();
  elapsedTime = (currentTime - previousTime) / 1000;

  accAngleX = (atan(accY / sqrt(pow(accX, 2) + pow(accZ, 2))) * 180 / PI) - errorAccX; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan(-1 * accX / sqrt(pow(accY, 2) + pow(accZ, 2))) * 180 / PI) - errorAccY; // AccErrorY ~(-1.58)

  gyroAngleX += gyroX * elapsedTime;
  gyroAngleY += gyroY * elapsedTime;
  gyroAngleZ += gyroZ * elapsedTime;

  yaw =  yaw + gyroZ * elapsedTime;
  // Complementary filter - combine acceleromter and gyro angle values
  roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
  pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
}

void loop() {
  getImu();

  if (counter % PRINT_RATE == 0){
    Serial.print("aX = "); Serial.print(accX, 2);
    Serial.print(" | aY = "); Serial.print(accY, 2);
    Serial.print(" | aZ = "); Serial.print(accZ, 2);
    Serial.print(" | tmp = "); Serial.print(temperature, 2);
    Serial.print(" | gX = "); Serial.print(gyroX, 2);
    Serial.print(" | gY = "); Serial.print(gyroY, 2);
    Serial.print(" | gZ = "); Serial.print(gyroZ, 2);
    Serial.print(" | angleX = "); Serial.print(gyroAngleX, 2);
    Serial.print(" | angleY = "); Serial.print(gyroAngleY, 2);
    Serial.print(" | angleZ = "); Serial.print(gyroAngleZ, 2);
    // Print the values on the serial monitor
    Serial.print(" | roll = "); Serial.print(roll);
    Serial.print(" | pitch = "); Serial.print(pitch);
    Serial.print(" | yaw = "); Serial.println(yaw);
    Serial.println(); 
  }

  ++counter;
  delay(DELAY);
}