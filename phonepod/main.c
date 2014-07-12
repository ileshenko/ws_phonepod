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
 */
#include <msp430g2553.h>
#include <string.h>

#include "config_lib.h"
#include <msplib_common.h>
#include <timer_lib.h>
#include <leds.h>
#include <servo.h>
#include <buttons.h>
#include <step_motor.h>

#if 0
static void xdelay(void)
{
	volatile int i = 0xffff;
	volatile int j = 0x4;

	while(j--)
		while(i--);
}

static void udelay(void)
{
	volatile int i = 0xffff;

	while(i--);
}

#endif

void jack_init(void)
{
	P2SEL &= ~JACK_P2;							// switch to GPIO mode
	P2SEL2 &= ~JACK_P2;							// switch to GPIO mode

	P2DIR &= ~JACK_P2;							// Set as Input
	P2REN |= JACK_P2;							// Poll Up/Down Resistor enable
	P2OUT |= JACK_P2;							// Poll Up
	P2IE |= JACK_P2;							// Interrupt Enabled
	P2IES |= JACK_P2;							// Hi/Lo edge
	P2IFG &= ~JACK_P2;							// IFG cleared
}
#if 0
typedef enum {
	TRACK_ERR = -1,
	TRACK_BW_F = 0, /* 500 Hz, 2000, 0x7d0 */
	TRACK_BW_S = 1, /* 1000 Hz, 1000, 0x3e8 */
	TRACK_STOP = 2, /* 1500 Hz, 667, 0x29b */
	TRACK_FW_S = 3, /* 2000 Hz, 500, 0x1f4 */
	TRACK_FW_F = 4, /* 2500 Hz, 400, 0x190 */
	TRACK_UP = 5, /* 3000 Hz, 333 */
	TRACK_DOWN = 6, /* 3500 Hz, 286 */
	TRACK_NOISE = 7, /* Noise on line. Save previous direction */
} track_mode_t;

static track_mode_t freq2track(unsigned short delta)
{
	if (delta == 0xCACA)
		return TRACK_NOISE;

	if (delta > (2000+500))
		return TRACK_ERR;
	if (delta > 1500)
		return TRACK_BW_F;
	if (delta > 833)
		return TRACK_BW_S;
	if (delta > ((667+500)>>1))
		return TRACK_STOP;
	if (delta > ((500+400)>>1))
		return TRACK_FW_S;
	if (delta > ((400+333)>>1))
		return TRACK_FW_F;
	if (delta > ((333+286)>>1))
		return TRACK_UP;
	if (delta > (286>>1))
		return TRACK_DOWN;
	return TRACK_ERR;
}
#endif

//static unsigned short prev_l, prev_r;
//static char stable_l, stable_r;
//track_mode_t track_l = TRACK_STOP, track_r = TRACK_STOP;
static unsigned char idxl, idxr;
static unsigned short deltsl[16];
static unsigned short deltsr[16];
static unsigned int freq_watchdog;

#pragma vector=PORT2_VECTOR
__interrupt void Port_2(void)
{
	static unsigned short prev_t0_l, prev_t0_r;
	unsigned short curr_tar = TA0R;

	freq_watchdog = 0;
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

#define STOP \
		xdelay(); \
		led_set(MLR, 0); \
		led_set(MRR, 0); \
		led_set(MLF, 0); \
		led_set(MRF, 0); \
		xdelay();

#if 0
static void set_trackl(track_mode_t mode)
{
	switch (mode)
	{
	case TRACK_NOISE:
		break;
	case TRACK_BW_F:
		led_set(MLR, 1);
		led_set(MLF, 0);
		led_set(TURBO, 1);
		break;
	case TRACK_BW_S:
		led_set(MLR, 1);
		led_set(MLF, 0);
		led_set(TURBO, 0);
		break;
	case TRACK_FW_S:
		led_set(MLR, 0);
		led_set(MLF, 1);
		led_set(TURBO, 0);
		break;
	case TRACK_FW_F:
		led_set(MLR, 0);
		led_set(MLF, 1);
		led_set(TURBO, 1);
		break;
	case TRACK_ERR:
	case TRACK_STOP:
	case TRACK_UP:
	case TRACK_DOWN:
	default:
		led_set(MLR, 0);
		led_set(MLF, 0);
	}
}

static void set_trackr(track_mode_t mode)
{
	switch (mode)
	{
	case TRACK_NOISE:
		break;
	case TRACK_BW_F:
		led_set(MRR, 1);
		led_set(MRF, 0);
		led_set(TURBO, 1);
		break;
	case TRACK_BW_S:
		led_set(MRR, 1);
		led_set(MRF, 0);
		led_set(TURBO, 0);
		break;
	case TRACK_FW_S:
		led_set(MRR, 0);
		led_set(MRF, 1);
		led_set(TURBO, 0);
		break;
	case TRACK_FW_F:
		led_set(MRR, 0);
		led_set(MRF, 1);
		led_set(TURBO, 1);
		break;
	case TRACK_ERR:
	case TRACK_STOP:
	case TRACK_UP:
	case TRACK_DOWN:
	default:
		led_set(MRR, 0);
		led_set(MRF, 0);
	}
}
#endif

void main(void)
{
//	unsigned int deltal, deltar;
//	track_mode_t new_model, new_moder;
//	int i, deviation;
	int srv2_up, angle = 90;
	int step_frw, step_pos = 140;

	WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT

	default_state();
	clock_init(); /* DCO, MCLK and SMCLK - 1MHz */
	timer_init();
	servo_init();
	leds_init();
	buttons_init();
	stepm_init();
	jack_init();
	leds_hello(LED);

	stepm_zeroise();
//	_EINT();

	stepm_set_pos(240);
	stepm_set_pos(120);
	stepm_set_pos(240);
	stepm_set_pos(20);
	stepm_set_pos(240);
	while (1)
	{
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

#if 0
		if (freq_watchdog++ > 0xff)
		{
			memset(deltsl, 0, sizeof(deltsl));
			memset(deltsr, 0, sizeof(deltsr));
		}

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
			deviation = deltal - deltsl[i];
			if (deviation < 20 && deviation > -20)
				continue;
			deltal = 0xCACA;
			break;
		}
		for (i = 0; i < 16; i++)
		{
			deviation = deltar - deltsr[i];
			if (deviation < 20 && deviation > -20)
				continue;
			deltar = 0xCACA;
			break;
		}

		new_model = freq2track(deltal);
		new_moder = freq2track(deltar);
		set_trackl(new_model);
		set_trackr(new_moder);
		if (new_model == TRACK_UP)
			servo_up();
		if (new_model == TRACK_DOWN)
			servo_down();
#endif

	}
}
