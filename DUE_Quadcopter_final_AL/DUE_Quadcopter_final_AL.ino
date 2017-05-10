/*
 * D2-D5 + D10,11 > RGB PWM out
 * D33, 31, 29, 27, 25, 23 > RC receiver in
 * D45 + D41,39,37 > ESC PWM out (hardware)
 * A0 > Light sens. voltage
 * A1-A3 > Battery voltage (3S)
 * D46, 48 > nRF24L01 CS CE out
 */
 
/***************** Hardware PWM definition **********************/ 
#include "pwm_lib.h"
using namespace arduino_due::pwm_lib;
#define PWM_PERIOD 250000                // 2,5 msecs period (400Hz refresh)
#define PWM_DUTY 100000                  // 1000 us duty
pwm<pwm_pin::PWMH6_PC18> pwm_pin45;
pwm<pwm_pin::PWMH1_PC5> pwm_pin37;
pwm<pwm_pin::PWMH3_PC9> pwm_pin41;
pwm<pwm_pin::PWMH2_PC7> pwm_pin39;

/********************* RC variables **************************/
volatile long CH1_Pulse, CH2_Pulse, CH3_Pulse, CH4_Pulse, CH5_Pulse, CH6_Pulse; 
volatile unsigned long time1r, time2r, time1n, time2n, time1g, time2g, time1y, time2y, time1A, time2A, current_time;
int esc_1, esc_2, esc_3, esc_4, start, throttle;

/*********************** MPU6050 ****************************/
#include "Wire.h"
int16_t gx, gy, gz;                               // raw values from gyro
long tempGX, tempGY, tempGZ;                      // averaging the gyro outputs
long Gyro_X, Gyro_Y, Gyro_Z;                      // roll, pitch, yaw (final dps)
int16_t gyroX_cal = -111.20, gyroY_cal = 65.51, gyroZ_cal = -13.45;
float mpu_gyro500_scale = 0.015267;               // scale factor for 500*/s gyro sensitivity (1/65.5)
int16_t temp;                                     // temperature raw
float Temp_MPU;                                   // temperature scaled to dgC

int16_t ax, ay, az;                             // raw accel
long tempAX, tempAY, tempAZ;                    // temp. var. for averaging the accel outputs
float Accel_X, Accel_Y, Accel_Z;                // raw accel output calculated to G-force
float accelX_cal = 0.02, accelY_cal = -0.02, accelZ_cal = -0.10;
float mpu_accel8_scale = 0.00024414;            // scale factor for +-8g accel range (1/4096)
float pitch, roll;                              // calculated degrees form accel
float roll_level_adjust, pitch_level_adjust;    // final values from AL to adjust the PID

/***************** RGB LED's ************************/
/*  #define RED_B 4           // PC26       0x4000000     
    #define RED_W 5           // PC25       0x2000000
    #define GREEN_B 3         // PC28       0x10000000
    #define GREEN_W 10        // PC29       0x20000000
    #define BLUE_B 2          // PB25       0x2000000
    #define BLUE_W 11         // PD7        0x80          */
#define BUILD_IN 13       // PB27       0x8000000
#define RED_BLUE 10       // PC+PD      PC 0x4000000 + PD 0x80
#define RED_full 20       // PC25+26    0x6000000
#define GREEN_full 30     // PC28+29    0x30000000
#define BLUE_full 40      // PB+PD      PB 0x2000000 + PD 0x80
#define PURPLE_full 50    //            PB 0x2000000 + PC 0x6000000 + PD 0x80
#define WHITE 60          // all        PB 0x2000000 + PC 0x36000000 + PD 0x80

/*********************Battery & Timer's ******************************/
#include <DueTimer.h>
volatile int duty, timer, mode;                               // pwm
unsigned long loop_time_start, loop_time_stop, loop_timer;    // void loop() timer var's (optional)
volatile int Cell1, Cell2, Cell3, battery_check_timer;        // battery temp's
float C1, C2, C3, battV;                                      // battery final values (averaged)
bool battOK, battLOW;                                         // battery issue flags
volatile bool ledON;                                          // RGB's on/off flag

/**************************** MS5611 **********************************/
#define MS5611_ADDR 0x77              // I2C address
#define MS5611_D1 0x40                // registers of the device
#define MS5611_D2 0x50
#define MS5611_RESET 0x1E
#define MS5611_D1D2_SIZE 3            // D1 and D2 result size (bytes)
#define MS5611_OSR_256 0x00           // OSR (Over Sampling Ratio) constants
#define MS5611_OSR_512 0x02
#define MS5611_OSR_1024 0x04
#define MS5611_OSR_2048 0x06
#define MS5611_OSR_4096 0x08
#define MS5611_PROM_BASE_ADDR 0xA2    // by adding ints from 0 to 6 we can read all the prom configuration values
#define MS5611_PROM_REG_COUNT 6       // number of registers in the PROM
#define MS5611_PROM_REG_SIZE 2        // size in bytes of a prom registry.
#define Conversion_Time 9050          // max time need to collect data at 4096 OSR
uint32_t pressCache, tempCache, C[MS5611_PROM_REG_COUNT];
bool MS5611_OK;
float Temp_MS, Pressure;

/************************* PID ****************************/
float pid_p_gain_roll = 1.52;                 //Gain setting for the roll P-controller
float pid_i_gain_roll = 0.015;                 //Gain setting for the roll I-controller
float pid_d_gain_roll = 1.0;                 //Gain setting for the roll D-controller
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = 1.57;              //Gain setting for the pitch P-controller
float pid_i_gain_pitch = 0.018;       //Gain setting for the pitch I-controller
float pid_d_gain_pitch = pid_d_gain_roll;       //Gain setting for the pitch D-controller
int pid_max_pitch = 400;                        //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 1.5;                  //Gain setting for the pitch P-controller
float pid_i_gain_yaw = 0.01;               //Gain setting for the pitch I-controller
float pid_d_gain_yaw = 0.0;                  //Gain setting for the pitch D-controller
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-)

float pid_error_temp, response_divider = 3.0;
float pid_i_mem_roll, pid_roll_setpoint, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, pid_output_yaw, pid_last_yaw_d_error;


void setup() 
{ Timer4.attachInterrupt(timer_pwm_handle).setFrequency(10000).start();        // 10 kHz pwm freq.
  Timer3.attachInterrupt(check_battery_voltage).setFrequency(1).start();       // 1 Hz at start
  
  PIOC->PIO_OER = 0x36000000;     // pins 3,4,5,10 output
  PIOB->PIO_OER = 0xA000000;      // pins 2,13 output
  PIOD->PIO_OER = 0x80;           // pin 11 output
  
  Wire.begin();                   // I2C lib. begin
  delay(25);
  Wire.setClock(350000L);         // 350kHz I2C transmision speed (default 100kHz)
  //Serial.begin(115200);

  Battery_init();                 // check if lipol cells voltage is ok (GREEN)
  MPU6050_init_clear();           // MPU6050 (gyro_accel_temp) initialization  (RED)
     
//  MS5611_init();                  // MS5611 (baro_temp) initialization  (BLUE)
    
  attachInterrupt(33, CH1_roll, CHANGE);        // read RC signals (interrupt on change)
  attachInterrupt(31, CH2_nick, CHANGE);
  attachInterrupt(29, CH3_gas, CHANGE);
  attachInterrupt(27, CH4_yaw, CHANGE);
  attachInterrupt(25, CH5_AUX1, CHANGE);
  attachInterrupt(23, CH6_AUX2, CHANGE);

  while(CH3_Pulse < 990 || CH3_Pulse > 1190 || CH4_Pulse < 1400)
  { delay(200);
    digitalWrite(13, HIGH);                         
    delay(200);
    digitalWrite(13, LOW); 
  }

  start = 0;
  CH4_Pulse = 1500;     // don't auto start motors after power up (bugfix)
  pwm_pin45.start(PWM_PERIOD,PWM_DUTY);       // 1197 start
  pwm_pin41.start(PWM_PERIOD,PWM_DUTY);       // 1201 start
  pwm_pin39.start(PWM_PERIOD,PWM_DUTY);       // 1198 start
  pwm_pin37.start(PWM_PERIOD,PWM_DUTY);       // 1201 start
}

void loop() 
{ //loop_time_start = micros();
  if (CH3_Pulse < 1190 && CH4_Pulse < 1050) start = 1;    // Motors start: throttle low and yaw left (step 1)
  if(start == 1 && CH3_Pulse < 1150 && CH4_Pulse > 1450)  // Yaw back to center position = start the motors (step 2)
  { start = 2;
    //Reset the pid controllers for a bumpless start.
    pid_i_mem_roll = 0;
    pid_last_roll_d_error = 0;
    pid_i_mem_pitch = 0;
    pid_last_pitch_d_error = 0;
    pid_i_mem_yaw = 0;
    pid_last_yaw_d_error = 0;
  }
  if(start == 2 && CH3_Pulse < 1190 && CH4_Pulse > 1950) start = 0;   // Motors stop: throttle low and yaw right.
  
  //MPU6050_read();
  MPU6050_read_clear();
      //  Temp_MS = MS5611_getTemperature(MS5611_OSR_4096);
      //  Pressure = MS5611_getPressure(MS5611_OSR_4096);
  DPS_RC();           // convert RC input signals to deg/s for PID and manipulate them for Auto-Level
  RGB_control();      // RGB visual effects control
  Battery_Check();    // calculate battery voltage and change Timer settings & RGB light if needed
  Calculate_PID();    // final PID calculations

  throttle = CH3_Pulse;                                                     //C3_Pulse changes very fast in interrupt, it must be saved temp. for this loop
  if (start == 2)                                                          //The motors are started.
    { if (throttle > 1800) throttle = 1800;                                   //We need some room to keep full control at full throttle.
      esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw - 3; //Calculate the pulse for esc 1 (front-right - CCW)
      esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw + 1; //Calculate the pulse for esc 2 (rear-right - CW)
      esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw - 2; //Calculate the pulse for esc 3 (rear-left - CCW)
      esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw + 1; //Calculate the pulse for esc 4 (front-left - CW)
    
      if (esc_1 < 1197) esc_1 = 1197;    // Keep the motors running
      if (esc_2 < 1201) esc_2 = 1201;      
      if (esc_3 < 1198) esc_3 = 1198;      
      if (esc_4 < 1201) esc_4 = 1201;         
    
      if (esc_1 > 2000) esc_1 = 2000;    // Limit the esc pulses to 2000us.
      if (esc_2 > 2000) esc_2 = 2000;       
      if (esc_3 > 2000) esc_3 = 2000;        
      if (esc_4 > 2000) esc_4 = 2000;
    }
  else
    { esc_1 = 1000;                    // If start is not 2 keep 1000us pulse for esc's.
      esc_2 = 1000;
      esc_3 = 1000;
      esc_4 = 1000;
    }

//RC_pulse_serial();
//Final_ESC_output_serial();
//BatteryV_serial();
//MPU6050_gyro_serial();
//MPU6050_accel_serial();
//MPU6050_serial_FULL();

//  loop_time_stop = micros();
//  Serial.println(loop_time_stop-loop_time_start);
                                   
  while (micros() - loop_timer < 2500);                  //loop slow down to 400Hz (default speed 1,6kHz)
  loop_timer = micros();                           //Set the timer for the next loop.
    
    pwm_pin45.set_duty(esc_1*100);        // Update pwm duty
    pwm_pin41.set_duty(esc_2*100);
    pwm_pin39.set_duty(esc_3*100);
    pwm_pin37.set_duty(esc_4*100);
}

