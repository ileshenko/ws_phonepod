/*
 * main.c
 * P1.0 NC
 * P1.1 UART_RX
 * P1.2 UART_TX
 * P1.3 StepM OnOff
 * P1.4 StepM B-
 * P1.5 StepM B+
 * P1.6 AJack MIC
 * P1.7 NC
 * P2.0 StepM A-
 * P2.1 StepM A+
 * P2.2 SRV1 (JP7)
 * P2.3 StepM ZERO
 * P2.4 LED
 * P2.5 SRV2 (JP6)
 * P2.6 AJack LEFT_D
 * P2.7 AJack RIGHT_D
 *
 * Clocks:
 * DCO 8 MHz
 * Master Clock MCLK - 8 MHz
 * Sub master  SMCLK - 8 MHz
 *
 * Timers:
 * TA (TA0) 1 MHz
 * TB (TA1) 1 MHz
 *
 * Usage of timers:
 * TA0:
 *    count continuous
 *    interrupts - NO
 *    used for measure Audio Input intervals (tones period)
 * TA1:
 *    count up to 20000 (20 ms)
 *    interrupts YES
 *    interrupts  CCR0 - RTC (50 HZ)
 *    CCR1 & CCR2 - for SERVO PWM
 *
 */
#include <msp430g2553.h>
#include <string.h>
#include <stdlib.h>

#include "config_lib.h"
#include <msplib_common.h>
#include <timer_lib.h>
#include <leds.h>
#include <servo.h>
#include <buttons.h>
#include <step_motor.h>

void jack_init(void)
{
	P2SEL &= ~JACK_P2;							// switch to GPIO mode
	P2SEL2 &= ~JACK_P2;							// switch to GPIO mode

	P2DIR &= ~JACK_P2;							// Set as Input
//	P2REN |= JACK_P2;							// Poll Up/Down Resistor enable
//	P2OUT |= JACK_P2;							// Poll Up
	P2IE |= JACK_P2;							// Interrupt Enabled
	P2IES |= JACK_P2;							// Hi/Lo edge
	P2IFG &= ~JACK_P2;							// IFG cleared
}

static unsigned char idxl, idxr;
static unsigned short deltsl[16];
static unsigned short deltsr[16];

#pragma vector=PORT2_VECTOR
__interrupt void Port_2(void)
{
	static unsigned short prev_t0_l, prev_t0_r;
	unsigned short curr_tar = TA0R;

	if (P2IFG & JACK_L)
	{
		P2IFG &= ~JACK_L;
		deltsl[idxl++&0xf] = curr_tar - prev_t0_l;
		prev_t0_l = curr_tar;
	}

	if (P2IFG & JACK_R)
	{
		P2IFG &= ~JACK_R;
		deltsr[idxr++&0xf] = curr_tar - prev_t0_r;
		prev_t0_r = curr_tar;
	}
}

void main(void)
{
	unsigned long deltal, deltar;
	int i, val, freq;
	int srv2_up, angle = 90;
	int step_frw, step_pos = 140;

	WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT

	default_state();
	clock_init(); /* DCO, MCLK and SMCLK - 1MHz */
	timer_init();
	servo_init();
	leds_init();
	buttons_init();
	stepm_init(); /* stepper motor depends on LEDs and Buttons */
	jack_init();
	leds_hello(LED);

	stepm_zeroise();
	_EINT();

	stepm_set_pos(240);
	servo_set(SRV2, SRV_ANGLE_MIN);

	stepm_set_pos(120);
	stepm_set_pos(240);
	servo_set(SRV2, SRV_ANGLE_MAX);

	stepm_set_pos(20);
	stepm_set_pos(240);
	servo_set(SRV2, 900);

	while (1)
	{
#if 0 /* demonstration */
		if (srv2_up)
		{
			angle += 16;
			if (angle >= SRV_ANGLE_MAX)
				srv2_up = 0;
		}
		else
		{
			angle -= 16;
			if (angle <= SRV_ANGLE_MIN)
				srv2_up = 1;
		}
		servo_set(SRV2, angle);

		if (step_frw)
		{
			step_pos--;
			if (step_pos <= 40)
				step_frw = 0;
		}
		else
		{
			step_pos++;
			if (step_pos >= STEPM_POS_MAX)
				step_frw = 1;
		}
		stepm_set_pos(step_pos);

//		rtc_sleep_for(1);
#endif

		deltal = deltar = 0;
		for (i = 0; i < 16; i++)
		{
			deltal += deltsl[i];
			deltar += deltsr[i];
		}
		deltal >>= 4;
		deltar >>= 4;
		for (i = 0; i < 16; i++)
		{
			if (abs(deltal - deltsl[i]) > 20)
			{
				deltal = 0;
				break;
			}
		}
		for (i = 0; i < 16; i++)
		{
			if (abs(deltar - deltsr[i]) > 20)
			{
				deltar = 0;
				break;
			}
		}

		if (deltal)
		{
			freq = 1000000ul/deltal;
			/* left line servo. 400 Hz for 0, 4000 Hz for 180 */
			val = (freq >> 1) - 200;
			if (abs(val - angle) > 10)
			{
				angle = val;
				servo_set(SRV2, angle);
			}
		}

		if (deltar)
		{
			freq = 1000000ul/deltar;
			/* Right line stepper. 400 Hz for (240 - 0), 4400 Hz for (240 - 200) */
			val = 240 - ((freq/20) - 20);
			if (abs(val - step_pos) > 10)
			{
				step_pos = val;
				stepm_set_pos(step_pos);
			}
		}
		rtc_sleep();
	}
}
