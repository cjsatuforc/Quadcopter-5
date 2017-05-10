///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Subroutine for reading the gyro
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void gyro_signalen(){
  Wire.beginTransmission(105);                                 //Start communication with the gyro (adress 1101001)
  Wire.write(168);                                             //Start reading @ register 28h and auto increment with every read
  Wire.endTransmission();                                      //End the transmission
  Wire.requestFrom(105, 6);                                    //Request 6 bytes from the gyro
  while(Wire.available() < 6);                                 //Wait until the 6 bytes are received
  lowByte = Wire.read();                                       //First received byte is the low part of the angular data
  highByte = Wire.read();                                      //Second received byte is the high part of the angular data
  gyro_roll = ((highByte<<8)|lowByte);                         //Multiply highByte by 256 (shift left by 8) and ad lowByte
  if(cal_int == 2000)gyro_roll -= gyro_roll_cal;               //Only compensate after the calibration
  lowByte = Wire.read();                                       //First received byte is the low part of the angular data
  highByte = Wire.read();                                      //Second received byte is the high part of the angular data
  gyro_pitch = ((highByte<<8)|lowByte);                        //Multiply highByte by 256 (shift left by 8) and ad lowByte
  gyro_pitch *= -1;                                            //Invert axis
  if(cal_int == 2000)gyro_pitch -= gyro_pitch_cal;             //Only compensate after the calibration
  lowByte = Wire.read();                                       //First received byte is the low part of the angular data
  highByte = Wire.read();                                      //Second received byte is the high part of the angular data
  gyro_yaw = ((highByte<<8)|lowByte);                          //Multiply highByte by 256 (shift left by 8) and ad lowByte
  gyro_yaw *= -1;                                              //Invert axis
  if(cal_int == 2000)gyro_yaw -= gyro_yaw_cal;                 //Only compensate after the calibration
}

