/*#include "pwm_lib.h"
using namespace arduino_due::pwm_lib;

#define PWM_PERIOD 290000 // 2,5 msecs (400Hz)
#define PWM_DUTY 100000 // 1000 usecs in hundredth of usecs (1e-8 secs)
pwm<pwm_pin::PWMH6_PC18> pwm_pin45;
pwm<pwm_pin::PWMH1_PC5> pwm_pin37;
pwm<pwm_pin::PWMH3_PC9> pwm_pin41;
pwm<pwm_pin::PWMH2_PC7> pwm_pin39;
*/
/*
 * D2 - D7 > RGB PWM
 * D19 > GY-86 INT pin
 * D33 - D23 > receiver line (6x)
 * D45 + D41,39,37 > ESC PWM OUT (hardware)
 */
uint32_t  chan;
volatile long CH1_Pulse, CH2_Pulse, CH3_Pulse, CH4_Pulse; 
volatile unsigned long time1r, time2r, time1n, time2n, time1g, time2g, time1y, time2y, current_time;

void setup() 
{// Serial.begin(115200);           //So you can display value on terminal screen
  PIOC->PIO_OER = 0x36000000;       // set pins 3,4,5,10 as output
  PIOB->PIO_OER = 0xA000000;        // set pins 2,13 as output
  pinMode(11, OUTPUT);
  
  attachInterrupt(33, CH1_roll, CHANGE);        // read receiver signals (interrupt)
  attachInterrupt(31, CH2_nick, CHANGE);
  attachInterrupt(29, CH3_gas, CHANGE);
  attachInterrupt(27, CH4_yaw, CHANGE);
/*
  ///Wait until the receiver is active and the throtle is set to the lower position.
  while(CH3_Pulse < 990 || CH3_Pulse > 1190 || CH4_Pulse < 1400)
  { delay(250);
    digitalWrite(13, HIGH);                         
    delay(250);
    digitalWrite(13, LOW); 
  }
*/  chan = g_APinDescription[45].ulPWMChannel;
    InitPIO();
    InitPWMController_MCLK();
}

void loop() 
{
//  print_value();
  

}

void InitPIO()
{
    //Because we are using PORTC.PIN23 in peripheral B mode
    //  we need to enable the clock for that line.
    PMC->PMC_PCER0 = bit(ID_PIOC);
     
    //Disable PIO Control on PC23 and set up for Peripheral B PWML7 
    //PWM Pin 6 on the board
    PIOC->PIO_PDR = PIO_PC18;
    PIOC->PIO_ABSR = PIO_PC18;
     
    //Enable output on C23
    PIOC->PIO_OER = PIO_PC18;
     
    //Enable pull-up on pin
    PIOC->PIO_PUDR = PIO_PC18;
}

void InitPWMController_MCLK()
{
    //Enable the PWM clock (36)
    PMC->PMC_PCER1 = bit((ID_PWM - 32));
     
    //Channel Prescaler - Use ClockA as configured in PWM_CLK
    //Preliminary frequency = 84000000 / 16 = 5250000Hz
    PWM->PWM_CH_NUM[chan].PWM_CMR = PWM_CMR_CPRE_MCK_DIV_1024;
     
    //Final frequency = 5250000 / 400 = 13125Hz
    SetPeriod(300);      // 400Hz
     
    //Duty cycle = (128 / 400) * 100 = 32%
    SetDuty(200);
     
    PWM->PWM_ENA = bit(1);
}

void SetPeriod(uint16_t period)
{
    //If the channel is disabled then we can write directly to the register
    //  else if enabled we write to the update register which acts as a double buffer
    if ((PWM->PWM_SR & bit(chan)) == 0)
        PWM->PWM_CH_NUM[chan].PWM_CPRD = period;
    else
        PWM->PWM_CH_NUM[chan].PWM_CPRDUPD = period;
}

void SetDuty(uint16_t duty)
{
    //If the channel is disabled then we can write directly to the register
    //  else if enabled we write to the update register which acts as a double buffer
    if ((PWM->PWM_SR & bit(chan)) == 0)
        PWM->PWM_CH_NUM[chan].PWM_CDTY = duty;
    else
        PWM->PWM_CH_NUM[chan].PWM_CDTYUPD = duty;
}

void print_value(){
  Serial.print("Roll: ");
  Serial.print(CH1_Pulse);
  Serial.print("\t");
  Serial.print("Nick: ");
  Serial.print(CH2_Pulse);
  Serial.print("\t");
  Serial.print("Gas: ");
  Serial.print(CH3_Pulse);
  Serial.print("\t");
  Serial.print("Yaw: ");
  Serial.println(CH4_Pulse);
  /*Serial.print("\t");
  Serial.print("1C: ");
  Serial.print(analogRead(A1)*(4.2 / 1023.0));
  Serial.print("\t");
  Serial.print("2C: ");
  Serial.print(analogRead(A2)*(8.4 / 1023.0));
  Serial.print("\t");
  Serial.print("3C: ");
  Serial.print(analogRead(A3)*(12.6 / 1023.0));
  Serial.print("\t");
  Serial.print("Light: ");
  Serial.println(analogRead(A0));
  */
}

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

