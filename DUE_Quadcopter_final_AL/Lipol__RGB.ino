void Battery_init()
{ PWM_write(GREEN_full, 30);
  delay(300);                                                 // delay for longer RGB effect + better ESC startup
  C1 = analogRead(A1) * (4.2 / 1023.0) + 0.04;                // fixed voltage dividor accuracy
  C2 = analogRead(A2) * (4.2 / 1023.0) + 0.02;
  C3 = analogRead(A3) * (4.2 / 1023.0);
  if (C1 < 3.3 || C2 < 3.3 || C3 < 3.3)                       // under this voltage don't let the quad start
    { battOK = false; battLOW = true; }
  else
    battOK = true;
  while (!battOK)                                             // infinite loop if battery too LOW
  { PWM_write(GREEN_full, 0); 
    delay(250);
    PWM_write(GREEN_full, 40);
    delay(250);
  }
  PWM_write(GREEN_full, 0);
  delay(150);
}

void check_battery_voltage()
{ Cell1 += analogRead(A1);
  Cell2 += analogRead(A2);
  Cell3 += analogRead(A3);
  battery_check_timer++;
  ledON = !ledON;
}

void Battery_Check()
{ if (battery_check_timer >= 6)                                     // after few measurements average the results
  { C1 = (Cell1/battery_check_timer) * (4.2 / 1023) + 0.04;        // fixing voltage dividor accuracy
    C2 = (Cell2/battery_check_timer) * (4.2 / 1023) + 0.02;        
    C3 = (Cell3/battery_check_timer) * (4.2 / 1023);  
    battV = C1+C2+C3; 
    if (C1 < 3.45 || C2 < 3.45 || C3 < 3.45)         // battery starts to be discharged
      battOK = false;
    if (C1 < 3.35 || C2 < 3.35 || C3 < 3.35)      // battery very low discharged
      battLOW = true;
    else
      { battOK = true; battLOW = false; }
    Cell1 = 0;  Cell2 = 0;  Cell3 = 0;            // clear battery temp.var's
    battery_check_timer = 0;                      // reset timer
    if (!battOK && battLOW)
      Timer3.setFrequency(3).start();             // if battery very low - speed up led blink and measurements
  } 
}

void RGB_control()
{ int light = analogRead(A0);
    
  if (CH5_Pulse <= 1050)                                            // SW default pos.
    { if (light >= 950 && battOK)
        PWM_write(RED_BLUE, 13);  
      else if (light <= 900 && battOK)
        PWM_write(RED_BLUE, 0);
      else if (ledON && !battOK)
        PWM_write(RED_BLUE, 40);
      else if (!ledON && !battOK)
        PWM_write(RED_BLUE, 0);
    }
  else if (CH5_Pulse >= 1450 && CH5_Pulse <= 1550)                  // SW middle pos.
    PWM_write(WHITE, 0);
  else if (CH5_Pulse >= 1950)                                       // SW max pulled down
    { if (battOK)
        PWM_write(WHITE, 60);
      else if (ledON && !battOK)
        PWM_write(WHITE, 45);
      else if (!ledON && !battOK)
        PWM_write(WHITE, 0);
    }
}

void timer_pwm_handle()
{ duty -= 1;
  if (duty == -1)
    duty = 100;

  if (duty == timer && timer != 0)
    { switch(mode)
        { case RED_BLUE:
            PIOC->PIO_SODR = 0x4000000;
            PIOD->PIO_SODR = 0x80;
            break;
          case BUILD_IN:
            PIOB->PIO_SODR = 0x8000000;
            break;
          case RED_full:
            PIOC->PIO_SODR = 0x6000000;
            break;
          case GREEN_full:
            PIOC->PIO_SODR = 0x30000000;
            break;
          case BLUE_full:
            PIOB->PIO_SODR = 0x2000000;
            PIOD->PIO_SODR = 0x80;
            break;
          case PURPLE_full:
            PIOB->PIO_SODR = 0x2000000;
            PIOC->PIO_SODR = 0x6000000;
            PIOD->PIO_SODR = 0x80;
            break;
          case WHITE:
            PIOB->PIO_SODR = 0x2000000;
            PIOC->PIO_SODR = 0x36000000;
            PIOD->PIO_SODR = 0x80;
            break;
        }
    }
  else if (duty == 0)
    { switch(mode)
        { case RED_BLUE:
            PIOC->PIO_CODR = 0x4000000;
            PIOD->PIO_CODR = 0x80;
            break;
          case BUILD_IN:
            PIOB->PIO_CODR = 0x8000000;
            break;
          case RED_full:
            PIOC->PIO_CODR = 0x6000000;
            break;
          case GREEN_full:
            PIOC->PIO_CODR = 0x30000000;
            break;
          case BLUE_full:
            PIOB->PIO_CODR = 0x2000000;
            PIOD->PIO_CODR = 0x80;
            break;
          case PURPLE_full:
            PIOB->PIO_CODR = 0x2000000;
            PIOC->PIO_CODR = 0x6000000;
            PIOD->PIO_CODR = 0x80;
            break;
          case WHITE:
            PIOB->PIO_CODR = 0x2000000;
            PIOC->PIO_CODR = 0x36000000;
            PIOD->PIO_CODR = 0x80;
            break;
        }
    }
}

void PWM_write(int function, int time_delay)
{  timer = time_delay;
   mode = function;
}
