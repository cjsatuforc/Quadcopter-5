void RC_pulse_serial()
{ Serial.print("RC: CH1,2,3,4,5,6> ");
  Serial.print(CH1_Pulse);
  Serial.print("\t");
  Serial.print(CH2_Pulse);
  Serial.print("\t");
  Serial.print(CH3_Pulse);
  Serial.print("\t");
  Serial.print(CH4_Pulse);
  Serial.print("\t");
  Serial.print(CH5_Pulse);
  Serial.print("\t");
  Serial.println(CH6_Pulse);
}

void BatteryV_serial()
{ Serial.print("Batt C1: ");
  Serial.print(C1);
  Serial.print(" C2: ");
  Serial.print(C2);
  Serial.print(" C3: ");
  Serial.print(C3);
  Serial.print(" counter: ");
  Serial.print(battery_check_timer);
  Serial.print(" T1: ");
  Serial.print(Cell1);
  Serial.print(" T2: ");
  Serial.print(Cell2);
  Serial.print(" T3: ");
  Serial.println(Cell3);
/*  Serial.print("Complete: ");
  Serial.println(battV);
*/
}

void MPU6050_gyro_serial()
{ Serial.print("gyro: ");
  Serial.print(Gyro_X);
  Serial.print("  ");
  Serial.print(Gyro_Y);
  Serial.print("  ");
  Serial.print(Gyro_Z);
  Serial.print("   temp: ");
  Serial.println(Temp_MPU);
}

void MPU6050_accel_serial()
{ Serial.print("accel: ");
  Serial.print(Accel_X);
  Serial.print("\t");
  Serial.print(Accel_Y);
  Serial.print("\t");
  Serial.print(Accel_Z);
  Serial.print("  angles: ");
  Serial.print(roll);
  Serial.print("\t");
  Serial.println(pitch);
}

void MPU6050_serial_FULL()
{ Serial.print("G_raw: ");
  Serial.print(gx);
  Serial.print(" ");
  Serial.print(gy);
  Serial.print(" ");
  Serial.println(gz);
  delay(50);
  Serial.print("G_calc: ");
  Serial.print(Gyro_X);
  Serial.print(" ");
  Serial.print(Gyro_Y);
  Serial.print(" ");
  Serial.println(Gyro_Z);
  delay(50);
  Serial.print("A_raw: ");
  Serial.print(ax);
  Serial.print(" ");
  Serial.print(ay);
  Serial.print(" ");
  Serial.println(az); 
  delay(50);
  Serial.print("A_calc: ");
  Serial.print(Accel_X);
  Serial.print(" ");
  Serial.print(Accel_Y);
  Serial.print(" ");
  Serial.println(Accel_Z);
  delay(50);
  Serial.print("angles: ");
  Serial.print(roll);
  Serial.print(" ");
  Serial.println(pitch);
}
void MS5611_serial()
{ Serial.print("baro> temp: ");
  Serial.print(Temp_MS);
  Serial.print("\t");
  Serial.print("press: ");
  Serial.println(Pressure);
}

void Final_ESC_output_serial()
{ Serial.print("ESC 1,2,3,4>   ");
  Serial.print(esc_1);
  Serial.print("\t");
  Serial.print(esc_2);
  Serial.print("\t");
  Serial.print(esc_3);
  Serial.print("\t");
  Serial.println(esc_4);
}

