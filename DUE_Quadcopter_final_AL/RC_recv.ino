void CH1_roll()
{ current_time = micros();
  if ((PIOC->PIO_PDSR & PIO_PC1) == PIO_PC1)
    time1r = current_time;
  else
    CH1_Pulse = current_time - time1r;
}
void CH2_nick()
{ current_time = micros();
  if ((PIOA->PIO_PDSR & PIO_PA7) == PIO_PA7)
    time1n = current_time;
  else
    CH2_Pulse = current_time - time1n;
}
void CH3_gas()
{ current_time = micros();
  if ((PIOD->PIO_PDSR & PIO_PD6) == PIO_PD6)
    time1g = current_time;
  else
    CH3_Pulse = current_time - time1g; 
}
void CH4_yaw()
{ current_time = micros();
  if ((PIOD->PIO_PDSR & PIO_PD2) == PIO_PD2)
    time1y = current_time;
  else
    CH4_Pulse = current_time - time1y;
}
void CH5_AUX1()
{ current_time = micros();
  if ((PIOD->PIO_PDSR & PIO_PD0) == PIO_PD0)
    time1A = current_time;
  else
    CH5_Pulse = current_time - time1A;
}
void CH6_AUX2()
{ current_time = micros();
  if ((PIOA->PIO_PDSR & PIO_PA14) == PIO_PA14)
    time2A = current_time;
  else
    CH6_Pulse = current_time - time2A;
}
