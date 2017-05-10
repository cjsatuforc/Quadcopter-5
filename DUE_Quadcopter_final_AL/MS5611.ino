void MS5611_init() {
  PWM_write(BLUE_full, 30);
  delay(500);
  Wire.beginTransmission(MS5611_ADDR);  
  Wire.write(MS5611_RESET);   // reset the device to populate its internal PROM registers
  Wire.endTransmission();
  delay(10);                  // some safety time
  for (int i=0; i<MS5611_PROM_REG_COUNT; i++) {
    Wire.beginTransmission(MS5611_ADDR);
    Wire.write(MS5611_PROM_BASE_ADDR + (i * MS5611_PROM_REG_SIZE));
    Wire.endTransmission();
    Wire.beginTransmission(MS5611_ADDR);
    Wire.requestFrom(MS5611_ADDR, (uint8_t) MS5611_PROM_REG_SIZE);
    if(Wire.available()) 
      { C[i] = Wire.read() << 8 | Wire.read();  
        MS5611_OK = true; }
  }
  while(!MS5611_OK)
  { PWM_write(BLUE_full, 40); 
    delay(250);
    PWM_write(BLUE_full, 0);
    delay(250);
  }
  PWM_write(BLUE_full, 0); 
  delay(200);
}      

float MS5611_getPressure(uint8_t OSR) {
  int32_t dT = MS5611_getDeltaTemp(OSR);
  uint32_t rawPress = MS5611_rawPressure(OSR);
  int64_t off  = ((uint32_t)C[1] <<16) + (((int64_t)dT * C[3]) >> 7);
  int64_t sens = ((uint32_t)C[0] <<15) + (((int64_t)dT * C[2]) >> 8);
  return ((( (rawPress * sens ) >> 21) - off) >> 15) / 100.0;
}

float MS5611_getTemperature(uint8_t OSR) {
  int64_t dT = MS5611_getDeltaTemp(OSR);
    return (2000 + ((dT * C[5]) >> 23)) / 100.0;
}

int32_t MS5611_getDeltaTemp(uint8_t OSR) {
  uint32_t rawTemp = MS5611_rawTemperature(OSR);
    return (int32_t)(rawTemp - ((uint32_t)C[4] << 8));
}

uint32_t MS5611_rawTemperature(uint8_t OSR) {
    MS5611_startConversion(MS5611_D2 + OSR);
    delayMicroseconds(Conversion_Time);    // need wait up to 9.04ms for sensor ADC
    tempCache = MS5611_getConversion(MS5611_D2 + OSR);
    return tempCache;
}

uint32_t MS5611_rawPressure(uint8_t OSR) {
    MS5611_startConversion(MS5611_D1 + OSR);
    delayMicroseconds(Conversion_Time);    // need wait up to 9.04ms for sensor ADC
    pressCache = MS5611_getConversion(MS5611_D1 + OSR);
    return pressCache;
}

void MS5611_startConversion(uint8_t command) {  // initialize pressure conversion
  Wire.beginTransmission(MS5611_ADDR);
  Wire.write(command);
  Wire.endTransmission();
}

uint32_t MS5611_getConversion(uint8_t command) {
  union { uint32_t val; uint8_t raw[4]; } conversion;
  // start read sequence
  Wire.beginTransmission(MS5611_ADDR);
  Wire.write(0);
  Wire.endTransmission();
  Wire.beginTransmission(MS5611_ADDR);
  Wire.requestFrom(MS5611_ADDR, (uint8_t) MS5611_D1D2_SIZE);
  if(Wire.available()) {
    conversion.raw[2] = Wire.read();
    conversion.raw[1] = Wire.read();
    conversion.raw[0] = Wire.read();
  }
  else 
    conversion.val = -1;
  return conversion.val;
}
