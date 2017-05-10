/***************** Hardware PWM definition **********************/ 
#include "pwm_lib.h"
using namespace arduino_due::pwm_lib;
#define PWM_PERIOD 250000                // 2,5 msecs period (400Hz refresh)
#define PWM_DUTY 100000                  // 1000 us duty
pwm<pwm_pin::PWMH6_PC18> pwm_pin45;
pwm<pwm_pin::PWMH1_PC5> pwm_pin37;
pwm<pwm_pin::PWMH3_PC9> pwm_pin41;
pwm<pwm_pin::PWMH2_PC7> pwm_pin39;

/*********************** MPU6050 ****************************/
#include "Wire.h"
int16_t gx, gy, gz;                         // raw  gyro
double gyroX_cal, gyroY_cal, gyroZ_cal;
double temp;                                // raw temperature 
int16_t ax, ay, az;                         // raw accel
double accelX_cal, accelY_cal, accelZ_cal;


void setup()
{ PIOC->PIO_OER = 0x36000000;     // pins 3,4,5,10 output
  PIOB->PIO_OER = 0xA000000;      // pins 2,13 output
  PIOD->PIO_OER = 0x80;           // pin 11 output
  
  PIOB->PIO_CODR = 0x2000000;     // all outputs LOW
  PIOC->PIO_CODR = 0x36000000;
  PIOD->PIO_CODR = 0x80;

  Wire.begin();                   // I2C lib. begin
  delay(25);
  Wire.setClock(400000L);         // 400kHz I2C transmision speed (default 100kHz)
  Serial.begin(115200);

  pwm_pin45.start(PWM_PERIOD,PWM_DUTY);       // 1197 start
  pwm_pin41.start(PWM_PERIOD,PWM_DUTY);       // 1201 start
  pwm_pin39.start(PWM_PERIOD,PWM_DUTY);       // 1198 start
  pwm_pin37.start(PWM_PERIOD,PWM_DUTY);       // 1201 start
  
  MPU6050_init();
}


void loop()
{
  MPU6050_calibrate();
  delay(200);
}




void MPU6050_init()
{ //Activate the MPU-6050
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x6B);                                                    //Send the requested starting register
  Wire.write(0x00);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1C);                                                    //Send the requested starting register
  Wire.write(0x10);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1B);                                                    //Send the requested starting register
  Wire.write(0x08);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Set Digital Low Pass Filter to ~43Hz
  Wire.beginTransmission(0x68);                                        //Start communication with the address found during search
  Wire.write(0x1A);                                                    //We want to write to the CONFIG register (1A hex)
  Wire.write(0x03);                                                    //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
  Wire.endTransmission();                                              //End the transmission with the gyro   
}  

void MPU6050_calibrate()
{ for (int i=0; i<2000; i++)
  { digitalWrite(13, HIGH);
    Wire.beginTransmission(0x68);             //Start communication with the gyro.
    Wire.write(0x3B);                         //Start reading @ register 43h and auto increment with every read.
    Wire.endTransmission();                   //End the transmission.
    Wire.requestFrom(0x68,14);                //Request 14 bytes from the gyro.
    
    while(Wire.available() < 14);             //Wait until the 14 bytes are received.
    ax = Wire.read()<<8|Wire.read();          //Add the low and high byte to the acc_x variable.
    ay = Wire.read()<<8|Wire.read();          //Add the low and high byte to the acc_y variable.
    az = Wire.read()<<8|Wire.read();          //Add the low and high byte to the acc_z variable.
    temp = Wire.read()<<8|Wire.read();        //Add the low and high byte to the temperature variable.
    gx = Wire.read()<<8|Wire.read();          //Read high and low part of the angular data.
    gy = Wire.read()<<8|Wire.read();          //Read high and low part of the angular data.
    gz = Wire.read()<<8|Wire.read();          //Read high and low part of the angular data.

    accelX_cal += ax;
    accelY_cal += ay;
    accelZ_cal += az;
    gyroX_cal += gx;
    gyroY_cal += gy;
    gyroZ_cal += gz;
    delay(2);
  }
  accelX_cal /= 2000;
  accelY_cal /= 2000;
  accelZ_cal /= 2000;
  gyroX_cal /= 2000;
  gyroY_cal /= 2000;
  gyroZ_cal /= 2000;

  Serial.print("Accel: ");
  Serial.print(accelX_cal);
  Serial.print("  ");
  Serial.print(accelY_cal);
  Serial.print("  ");
  Serial.print(accelZ_cal);
  Serial.print("  Gyro: ");
  Serial.print(gyroX_cal);
  Serial.print("  ");
  Serial.print(gyroY_cal);
  Serial.print("  ");
  Serial.println(gyroZ_cal);

  accelX_cal = 0;
  accelY_cal = 0;
  accelZ_cal = 0;
  gyroX_cal = 0;
  gyroY_cal = 0;
  gyroZ_cal = 0;
  digitalWrite(13, LOW);
}
