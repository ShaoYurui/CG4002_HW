#include "Wire.h" // This library allows you to communicate with I2C devices.

////////////////////////////// IMU PRE_COMPILE DEFINES //////////////////////////////////////
#define MPU_ADDR              0x68
#define SEND_FREQ             50 // Hz
#define LOOP_TIME             (1000/SEND_FREQ) // ms
/////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////// IMU  PREPROCESS DEFINES///////////////////////////////////////
#define CALIBRATE_SAMPLE_NUM  200
#define ACCE_SCALE_FACTOR     1000
#define GYRO_SCALE_FACTOR     1000
/////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////// HANDSHAKE PRE_COMPILE DEFINES ////////////////////////////////
#define REQUEST_H             'H'
#define ACK_H                 'A'
/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////// IMU RAW DATA VARIABLES //////////////////////////////////////
int16_t accelerometer_x, accelerometer_y, accelerometer_z; 
int16_t gyro_x, gyro_y, gyro_z; 
int16_t temperature; 
/////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////// IMU CALIBRATION VARIABLES ////////////////////////////////////
int16_t accelerometer_x_cal, accelerometer_y_cal, accelerometer_z_cal; 
int16_t gyro_x_cal, gyro_y_cal, gyro_z_cal; 
bool done_IMU_calibration = false;
/////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////// IMU PREPROCESS VARIABLES /////////////////////////////////////
float accelerometer_x_processed, accelerometer_y_processed, accelerometer_z_processed; 
float gyro_x_processed, gyro_y_processed, gyro_z_processed; 
/////////////////////////////////////////////////////////////////////////////////////////////

void setup() 
{
  Serial.begin(115200);//initial the Serial
  setup_IMU();
  calibrate_IMU();
}

void loop()
{
  read_IMU_data();
  preprocess_IMU_data();

  if(done_handshake())
  {
    Serial.print(accelerometer_x_processed);
    Serial.print(accelerometer_y_processed); 
    Serial.print(accelerometer_z_processed); 
    Serial.print(gyro_x_processed); 
    Serial.print(gyro_y_processed);
    Serial.print(gyro_z_processed);
  }

  delay(LOOP_TIME);
}

void setup_IMU()
{
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
}

void read_IMU_data()
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
  Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire.requestFrom(MPU_ADDR, 7*2, true); // request a total of 7*2=14 registers

  // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
  accelerometer_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  accelerometer_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  accelerometer_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
  temperature = Wire.read()<<8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
  gyro_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
  gyro_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
  gyro_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)
}

void calibrate_IMU()
{
  reset_cal_bias();

  for(int i = 0; i < CALIBRATE_SAMPLE_NUM; i++)
  {
    if(i % 25 == 0) // blink LED to indicate calibration in process
    {
      digitalWrite(13,!digitalRead(13));
    }

    read_IMU_data();

    accelerometer_x_cal += accelerometer_x;
    accelerometer_y_cal += accelerometer_y;
    accelerometer_z_cal += accelerometer_z;
    gyro_x_cal += gyro_x;
    gyro_y_cal += gyro_y;
    gyro_z_cal += gyro_z; 

    delay(10);
  }

  accelerometer_x_cal /= CALIBRATE_SAMPLE_NUM;
  accelerometer_y_cal /= CALIBRATE_SAMPLE_NUM;
  accelerometer_z_cal /= CALIBRATE_SAMPLE_NUM;
  gyro_x_cal /= CALIBRATE_SAMPLE_NUM;
  gyro_y_cal /= CALIBRATE_SAMPLE_NUM;
  gyro_z_cal /= CALIBRATE_SAMPLE_NUM; 

  done_IMU_calibration = true;
}

void reset_cal_bias()
{
  accelerometer_x_cal = 0;
  accelerometer_y_cal = 0;
  accelerometer_z_cal = 0;
  gyro_x_cal = 0;
  gyro_y_cal = 0;
  gyro_z_cal = 0; 
}

void preprocess_IMU_data()
{
  accelerometer_x_processed = (accelerometer_x - accelerometer_x_cal) / ACCE_SCALE_FACTOR;
  accelerometer_y_processed = (accelerometer_y - accelerometer_y_cal) / ACCE_SCALE_FACTOR;
  accelerometer_z_processed = (accelerometer_z - accelerometer_z_cal) / ACCE_SCALE_FACTOR;
  gyro_x_processed = (accelerometer_x - gyro_x_cal) / GYRO_SCALE_FACTOR;
  gyro_y_processed = (accelerometer_y - gyro_y_cal) / GYRO_SCALE_FACTOR;
  gyro_z_processed = (accelerometer_z - gyro_z_cal) / GYRO_SCALE_FACTOR;
}

bool done_handshake()
{
  char byte_in;
  if(Serial.available())
  {
    byte_in = Serial.read();
    if(byte_in == REQUEST_H) 
    {
      Serial.write(ACK_H);
      return true;
    } 
    else 
    {
      Serial.write(byte_in);
      return false;
    }
  }
}