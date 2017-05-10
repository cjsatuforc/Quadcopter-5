/*
void MPU6050_init()
{ PWM_write(RED_full, 30);   
  delay(500);
  mpu.initialize();
    mpu.setXGyroOffset(53);
    mpu.setYGyroOffset(-35);
    mpu.setZGyroOffset(5);
        mpu.setXAccelOffset(-83);     // -99    pitch
        mpu.setYAccelOffset(-2678);   // -2685    roll
        mpu.setZAccelOffset(1255);    // 1255   yaw
  mpuStatus = mpu.testConnection();
  while (!mpuStatus)
    { PWM_write(RED_full, 0);
      delay(250);
      PWM_write(RED_full, 40);
      delay(250);
    } 
  PWM_write(RED_full, 0);
  delay(200);
}

void MPU6050_read() 
{ /*while (av_count < 5)
  { mpu.getAcceleration(&ax, &ay, &az);                // read accel raw 
    tempAX += ax * mpu_accel8_scale;
    tempAY += ay * mpu_accel8_scale;
    tempAZ += az * mpu_accel8_scale;
    av_count++; 
  }
    Accel_X = tempAX / av_count;
    Accel_Y = tempAY / av_count;
    Accel_Z = tempAZ / av_count;
    av_count = 0;
    tempAX=0; tempAY=0; tempAZ=0;
  */
/*  mpu.getAcceleration(&ax, &ay, &az);            // read accel raw 
   Accel_X = ax * mpu_accel8_scale;
   Accel_Y = ay * mpu_accel8_scale;
   Accel_Z = az * mpu_accel8_scale;
*/
/*  mpu.getAcceleration(&ax, &ay, &az);            // read accel raw 
   Accel_X = (ax * alpha + (Accel_X * (1.0 - alpha))) * mpu_accel8_scale;
   Accel_Y = (ay * alpha + (Accel_Y * (1.0 - alpha))) * mpu_accel8_scale;
   Accel_Z = (az * alpha + (Accel_Z * (1.0 - alpha))) * mpu_accel8_scale;
      
  mpu.getRotation(&gx, &gy, &gz);                // read gyro raw decimal values...
   Gyro_X = gx * mpu_gyro500_scale;              // convert to dps...
   Gyro_Y = -1 * gy * mpu_gyro500_scale;         // and change directions if need        
   Gyro_Z = -1 * gz * mpu_gyro500_scale;  
   
  Temp_MPU = (mpu.getTemperature()/340.00+36.53)-4;       // read sensor temperture and convert to dgC

  roll  = (atan2(Accel_Y, Accel_Z)*180.0)/M_PI;
  pitch = (atan2(Accel_X, sqrt(Accel_Y*Accel_Y + Accel_Z*Accel_Z))*180.0)/M_PI;
  
  pitch_level_adjust = pitch * 15.0;                  //Calculate the pitch angle correction
  roll_level_adjust = roll * 15.0;                    //Calculate the roll angle correction

//  pitch_level_adjust = 0;                  //Calculate the pitch angle correction
//  roll_level_adjust = 0;                    //Calculate the roll angle correction

  Serial.print("angles: ");
  Serial.print(roll);
  Serial.print(" ");
  Serial.println(pitch);
  
}
*/
//////////////////////////////////////////////////////////////////////////////////////////////////
void MPU6050_init_clear()
{ PWM_write(RED_full, 30);   
  delay(300);
  //Activate the MPU-6050
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
  PWM_write(RED_full, 0);
  delay(150);
}  

void MPU6050_read_clear()
{ Wire.beginTransmission(0x68);             //Start communication with the gyro.
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

  Accel_X = (ax * mpu_accel8_scale) - accelX_cal;
  Accel_Y = (ay * mpu_accel8_scale) - accelY_cal;
  Accel_Z = (az * mpu_accel8_scale) - accelZ_cal;
  Temp_MPU = (temp/340.00+36.53)-2;
  Gyro_X = (gx - gyroX_cal) * mpu_gyro500_scale;              // substact the offset and convert to dps
  Gyro_Y = -1 * (gy - gyroY_cal) * mpu_gyro500_scale;         // also change directions if need        
  Gyro_Z = -1 * (gz - gyroZ_cal) * mpu_gyro500_scale;

  roll  = (atan2(Accel_Y, Accel_Z)*180.0)/M_PI;
  pitch = (atan2(Accel_X, sqrt(Accel_Y*Accel_Y + Accel_Z*Accel_Z))*180.0)/M_PI;

  /********** AUTO-LEVEL **********/
  if (CH6_Pulse < 1500)                           // Auto-Level default ON
  { pitch_level_adjust = pitch * 15.0;                  
    roll_level_adjust = roll * 15.0;                    
  }
  else
  { pitch_level_adjust = 0;                  
    roll_level_adjust = 0; 
  }                  
}

