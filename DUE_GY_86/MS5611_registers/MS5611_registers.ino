#include <Wire.h>

// I2C address
#define MS5611_ADDR 0x77 
// registers of the device
#define MS5611_D1 0x40
#define MS5611_D2 0x50
#define MS5611_RESET 0x1E
// D1 and D2 result size (bytes)
#define MS5611_D1D2_SIZE 3
// OSR (Over Sampling Ratio) constants
#define MS5611_OSR_256 0x00
#define MS5611_OSR_512 0x02
#define MS5611_OSR_1024 0x04
#define MS5611_OSR_2048 0x06
#define MS5611_OSR_4096 0x08
// by adding ints from 0 to 6 we can read all the prom configuration values
#define MS5611_PROM_BASE_ADDR 0xA2 
#define MS5611_PROM_REG_COUNT 6    // number of registers in the PROM
#define MS5611_PROM_REG_SIZE 2     // size in bytes of a prom registry.
#define Conversion_Time 9050       // max time need to collect data at 4096 OSR

uint32_t pressCache, tempCache, C[MS5611_PROM_REG_COUNT];
float temperature, pression;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  MS5611_init();
  //while (!MS5611_init());
}

void loop() {
  
  Serial.print("temp: ");
  
    temperature = MS5611_getTemperature(MS5611_OSR_4096);
  
  Serial.print(temperature);
  Serial.print(" degC pres: ");
  
    pression = MS5611_getPressure(MS5611_OSR_4096);
  
  Serial.print(pression);
  Serial.println(" mbar");

}


