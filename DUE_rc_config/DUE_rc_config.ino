#include "pwm_lib.h"
using namespace arduino_due::pwm_lib;

#define PWM_PERIOD 250000 // 2,5 msecs in hundredth of usecs (1e-8 secs)
#define PWM_DUTY 100000 // 1000 usecs in hundredth of usecs (1e-8 secs)
pwm<pwm_pin::PWMH6_PC18> pwm_pin45;
pwm<pwm_pin::PWMH1_PC5> pwm_pin37;
pwm<pwm_pin::PWMH3_PC9> pwm_pin41;
pwm<pwm_pin::PWMH2_PC7> pwm_pin39;

volatile int CH1_Pulse, CH2_Pulse, CH3_Pulse, CH4_Pulse; 
volatile float time1r, time2r, time1n, time2n, time1g, time2g, time1y, time2y;

void setup() 
{ Serial.begin(115200); //So you can display value on terminal screen
  
  attachInterrupt(33, CH1_roll, CHANGE);        // read receiver signals (interrupt)
  attachInterrupt(31, CH2_nick, CHANGE);
  attachInterrupt(29, CH3_gas, CHANGE);
  attachInterrupt(27, CH4_yaw, CHANGE);

  pwm_pin45.start(PWM_PERIOD,PWM_DUTY);
  pwm_pin37.start(PWM_PERIOD,PWM_DUTY);
  pwm_pin41.start(PWM_PERIOD,PWM_DUTY);
  pwm_pin39.start(PWM_PERIOD,PWM_DUTY);
}

void loop() 
{
  print_value();
}


void CH1_roll()
{ if (digitalRead(33)==HIGH)
    time1r = micros();
  else
  { time2r = micros();
    CH1_Pulse = time2r - time1r;  }
}
void CH2_nick()
{ if (digitalRead(31)==HIGH)
    time1n = micros();
  else
  { time2n = micros();
    CH2_Pulse = time2n - time1n;  }
}
void CH3_gas()
{ if (digitalRead(29)==HIGH)
    time1g = micros();
  else
  { time2g = micros();
    CH3_Pulse = time2g - time1g;  }
}
void CH4_yaw()
{ if (digitalRead(27)==HIGH)
    time1y = micros();
  else
  { time2y = micros();
    CH4_Pulse = time2y - time1y;  }
}


void print_value(){
  Serial.print("Roll: ");
  Serial.print(CH1_Pulse);
  
  Serial.print("  Nick: ");
  Serial.print(CH2_Pulse);
  
  Serial.print("  Gas: ");
  Serial.print(CH3_Pulse);
  
  Serial.print("  Yaw: ");
  Serial.println(CH4_Pulse);
}


void print_graph(){
  Serial.print("Roll:");
  if(CH1_Pulse - 1480 < 0)Serial.print("<<<");
  else if(CH1_Pulse - 1520 > 0)Serial.print(">>>");
  else Serial.print("-+-");
  Serial.print(CH1_Pulse);
  
  Serial.print("  Nick:");
  if(CH2_Pulse - 1480 < 0)Serial.print("vvv");
  else if(CH2_Pulse - 1520 > 0)Serial.print("^^^");
  else Serial.print("-+-");
  Serial.print(CH2_Pulse);
  
  Serial.print("  Gas:");
  if(CH3_Pulse - 1480 < 0)Serial.print("vvv");
  else if(CH3_Pulse - 1520 > 0)Serial.print("^^^");
  else Serial.print("-+-");
  Serial.print(CH3_Pulse);
  
  Serial.print("  Yaw:");
  if(CH4_Pulse - 1480 < 0)Serial.print("<<<");
  else if(CH4_Pulse - 1520 > 0)Serial.print(">>>");
  else Serial.print("-+-");
  Serial.println(CH4_Pulse);
}
