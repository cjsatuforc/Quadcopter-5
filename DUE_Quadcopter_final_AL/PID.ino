void DPS_RC()
{ /*  The PID set point in dps is determined by the receiver input
   *  In the case of deviding by 3 the max setpoint rate is 164 dps ((500-8)/3 = 164dps)
   *  Little 16us dead band for better results
   */
  pid_roll_setpoint = 0;
  if (CH1_Pulse > 1508)         pid_roll_setpoint = CH1_Pulse - 1508;
  else if (CH1_Pulse < 1492)    pid_roll_setpoint = CH1_Pulse - 1492;
  pid_roll_setpoint -= roll_level_adjust;          //Subtract the angle correction from the standardized receiver roll input value.
  pid_roll_setpoint /= response_divider;           //Divide the setpoint for the PID roll controller by 3 to get angles in degrees.
  
  pid_pitch_setpoint = 0;
  if (CH2_Pulse > 1508)         pid_pitch_setpoint = CH2_Pulse - 1508;
  else if (CH2_Pulse < 1492)    pid_pitch_setpoint = CH2_Pulse - 1492;
  pid_pitch_setpoint -= pitch_level_adjust;        //Subtract the angle correction from the standardized receiver pitch input value.
  pid_pitch_setpoint /= response_divider;          //Divide the setpoint for the PID pitch controller by 3 to get angles in degrees.

  pid_yaw_setpoint = 0;
  if (CH3_Pulse > 1150) //Do not yaw when turning off the motors.
  { if (CH4_Pulse > 1508)       pid_yaw_setpoint = (CH4_Pulse - 1508)/response_divider;
    else if (CH4_Pulse < 1492)  pid_yaw_setpoint = (CH4_Pulse - 1492)/response_divider;
  }
}

void Calculate_PID(){
  //Roll calculations (X)
  pid_error_temp = Gyro_X - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if (pid_i_mem_roll > pid_max_roll)             pid_i_mem_roll = pid_max_roll;
  else if (pid_i_mem_roll < pid_max_roll * -1)   pid_i_mem_roll = pid_max_roll * -1;
  
  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if (pid_output_roll > pid_max_roll)            pid_output_roll = pid_max_roll;
  else if (pid_output_roll < pid_max_roll * -1)  pid_output_roll = pid_max_roll * -1;
  
  pid_last_roll_d_error = pid_error_temp;
  
  //Pitch calculations  (Y)
  pid_error_temp = Gyro_Y - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if (pid_i_mem_pitch > pid_max_pitch)            pid_i_mem_pitch = pid_max_pitch;
  else if (pid_i_mem_pitch < pid_max_pitch * -1)  pid_i_mem_pitch = pid_max_pitch * -1;
  
  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if (pid_output_pitch > pid_max_pitch)           pid_output_pitch = pid_max_pitch;
  else if (pid_output_pitch < pid_max_pitch * -1) pid_output_pitch = pid_max_pitch * -1;
    
  pid_last_pitch_d_error = pid_error_temp;
    
  //Yaw calculations  (Z)
  pid_error_temp = Gyro_Z - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if (pid_i_mem_yaw > pid_max_yaw)            pid_i_mem_yaw = pid_max_yaw;
  else if (pid_i_mem_yaw < pid_max_yaw * -1)  pid_i_mem_yaw = pid_max_yaw * -1;
  
  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if (pid_output_yaw > pid_max_yaw)           pid_output_yaw = pid_max_yaw;
  else if (pid_output_yaw < pid_max_yaw * -1) pid_output_yaw = pid_max_yaw * -1;
    
  pid_last_yaw_d_error = pid_error_temp;
}

