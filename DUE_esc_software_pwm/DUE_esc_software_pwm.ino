volatile int CH1_Pulse, CH2_Pulse, CH3_Pulse, CH4_Pulse; 
volatile unsigned long time1r, time2r, time1n, time2n, time1g, time2g, time1y, time2y, current_time;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer, zero_timer;
int start, led, CH1, CH2, CH3, CH4;

void setup() 
{ pinMode(51, OUTPUT);           // ESC outputs
  pinMode(49, OUTPUT);
  pinMode(47, OUTPUT);
  pinMode(45, OUTPUT);
  pinMode(13, OUTPUT);
  
  attachInterrupt(33, CH1_roll, CHANGE); 
  attachInterrupt(31, CH2_nick, CHANGE);
  attachInterrupt(29, CH3_gas, CHANGE);
  attachInterrupt(27, CH4_yaw, CHANGE);

  ///Wait until the receiver is active and the throtle is set to the lower position.
  while(CH3_Pulse < 990 || CH3_Pulse > 1020 || CH4_Pulse < 1400)
  { start ++;                                        //While waiting increment start whith every loop.
    if(start == 125){                                //Every 125 loops (500ms).
      led = !led;
      digitalWrite(13, led);            //Change the led status.
      start = 0;                                     //Start again at 0.
    }
    delay(3); }
  start = 0;
  digitalWrite(13, LOW);                             //Turn off the led.
  zero_timer = micros();                             //Set the zero_timer for the first loop.
}

void loop() 
{
  while(zero_timer + 4000 > micros());        //Start the pulse after 4000 micro seconds.
  zero_timer = micros();                      //Reset the zero timer.
  CH1 = 1; CH2 = 1; CH3 = 1; CH4 = 1; 
  PIOC->PIO_SODR = 0x55000;                   //Set Ouput Data Register
  timer_channel_1 = CH3_Pulse + zero_timer;   //Calculate the time when digital port 8 is set low.
  timer_channel_2 = CH3_Pulse + zero_timer;   //Calculate the time when digital port 9 is set low.
  timer_channel_3 = CH3_Pulse + zero_timer;   //Calculate the time when digital port 10 is set low.
  timer_channel_4 = CH3_Pulse + zero_timer;   //Calculate the time when digital port 11 is set low.
  
  while(CH1 || CH2 || CH3 || CH4)
  { esc_loop_timer = micros();                               //Check the current time.
    if(timer_channel_1 <= esc_loop_timer) { PIOC->PIO_CODR = PIO_PC12; CH1 = 0; } //When the delay time is expired, digital port 8 is set low.
    if(timer_channel_2 <= esc_loop_timer) { PIOC->PIO_CODR = PIO_PC14; CH2 = 0; }//When the delay time is expired, digital port 9 is set low.
    if(timer_channel_3 <= esc_loop_timer) { PIOC->PIO_CODR = PIO_PC16; CH3 = 0; }//When the delay time is expired, digital port 10 is set low.
    if(timer_channel_4 <= esc_loop_timer) { PIOC->PIO_CODR = PIO_PC18; CH4 = 0; }//When the delay time is expired, digital port 11 is set low.
  }
}



void CH1_roll()
{ current_time = micros();
  if (digitalRead(33)==HIGH)
    time1r = current_time;
  else
    CH1_Pulse = current_time - time1r;
}
void CH2_nick()
{ current_time = micros();
  if (digitalRead(31)==HIGH)
    time1n = current_time;
  else
    CH2_Pulse = current_time - time1n;
}
void CH3_gas()
{ current_time = micros();
  if (digitalRead(29)==HIGH)
    time1g = current_time;
  else
    CH3_Pulse = current_time - time1g;
}
void CH4_yaw()
{ current_time = micros();
  if (digitalRead(27)==HIGH)
    time1y = current_time;
  else
    CH4_Pulse = current_time - time1y;
}


void print_signals(){
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
