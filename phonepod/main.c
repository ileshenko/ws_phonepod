/*
 * main.c
 * P1.0 NC
 * P1.1 UART_RX
 * P1.2 UART_TX
 * P1.3 K5
 * P1.4 K4
 * P1.5 K3
 * P1.6 MIC
 * P1.7 NC
 * P2.0 K2
 * P2.1 K1
 * P2.2 SRV1 (JP7)
 * P2.3 BTN
 * P2.4 LED
 * P2.5 SRV2 (JP6)
 * P2.6 LEFT_D
 * P2.7 RIGHT_D
 */
#include <msp430g2553.h>
#include <string.h>

#include "config_lib.h"
#include <msplib_common.h>
#include <timer_lib.h>
#include <leds.h>
#include <servo.h>

#if 1
static void xdelay(void)
{
	volatile int i = 0xfff;
	volatile int j = 0x1;

	while(j--)
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

#if 0
static void servo_up(void)
{
	unsigned int val = TA1CCR1;
	volatile int i = 0xfff;

	while(i--);

	if (++val > SERVO_180)
		val = SERVO_180;
	TA1CCR1 = val;
}

static void servo_down(void)
{
	unsigned int val = TA1CCR1;
	volatile int i = 0xfff;

	while(i--);

	if (--val < SERVO_0)
		val = SERVO_0;
	TA1CCR1 = val;
}
#endif

void main(void)
{
	unsigned int deltal, deltar;
	track_mode_t new_model, new_moder;
	int i, deviation;
	int srv2_up, angle = 90;

	WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT

	default_state();
	clock_init(); /* DCO, MCLK and SMCLK - 1MHz */
	timer_init();
	servo_init();
	leds_init();
	jack_init();
	leds_hello(LED);

	_EINT();
	while (1)
	{
		if (srv2_up)
		{
			angle++;
			if (angle >= SRV_ANGLE_MAX)
				srv2_up = 0;
		}
		else
		{
			angle--;
			if (angle <= SRV_ANGLE_MIN)
				srv2_up = 1;
		}
		servo_set(SRV2, angle);
		xdelay();

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
