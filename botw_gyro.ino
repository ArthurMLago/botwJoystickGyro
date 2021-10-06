
#include "MPU9250.h"

// an MPU9250 object with the MPU-9250 sensor on SPI bus 0 and chip select pin 10
MPU9250 IMU(SPI,10);
int status;

void setup() {
  pinMode(6, OUTPUT);
  digitalWrite(6, HIGH);
  // serial to display data
  Serial.begin(115200);
  while(!Serial) {}
  // start communication with IMU 
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {status = IMU.begin();Serial.println("failure loop");Serial.println(status);}
  }
  // setting DLPF bandwidth to 20 Hz
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  // setting SRD to 19 for a 50 Hz update rate
  IMU.setSrd(3);
  // enabling the data ready interrupt
  IMU.enableDataReadyInterrupt();

    IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
    IMU.setGyroRange(MPU9250::GYRO_RANGE_1000DPS);

  pinMode(1,INPUT);
  attachInterrupt(digitalPinToInterrupt(2),getIMU,RISING);

}

void loop() {}

void getIMU(){ 
  // read the sensor
  IMU.readSensor();
  // display the data
  int16_t placeholder;
  Serial.print("botw1");
  placeholder=IMU.getAccelX_mss();
  Serial.write((uint8_t*)&placeholder, 2);
  placeholder=(IMU.getAccelY_mss());
  Serial.write((uint8_t*)&placeholder, 2);
  placeholder=(IMU.getAccelZ_mss());
  Serial.write((uint8_t*)&placeholder, 2);
  placeholder=(IMU.getGyroX_rads());
  Serial.write((uint8_t*)&placeholder, 2);
  placeholder=(IMU.getGyroY_rads());
  Serial.write((uint8_t*)&placeholder, 2);
  placeholder=(IMU.getGyroZ_rads());
  Serial.write((uint8_t*)&placeholder, 2);
  placeholder=(IMU.getMagX_uT());
  Serial.write((uint8_t*)&placeholder, 2);
  placeholder=(IMU.getMagY_uT());
  Serial.write((uint8_t*)&placeholder, 2);
  placeholder=(IMU.getMagZ_uT());
  Serial.write((uint8_t*)&placeholder, 2);
}
