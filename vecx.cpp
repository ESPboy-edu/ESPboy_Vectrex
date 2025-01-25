#include <stdio.h>
#include "e6809.h"
#include "vecx.h"
#include "osint.h"
#include "e8910.h"
#include <Arduino.h>
#include "gbConfig.h"

 uint8_t *rom;
 uint8_t *cart;
 uint8_t *ram;

/* the sound chip registers */

uint8_t snd_regs[16];
static uint16_t snd_select;

/* the via 6522 registers */

static uint16_t via_ora;
static uint16_t via_orb;
static uint16_t via_ddra;
static uint16_t via_ddrb;
static uint16_t via_t1on;  /* is timer 1 on? */
static uint16_t via_t1int; /* are timer 1 interrupts allowed? */
static uint16_t via_t1c;
static uint16_t via_t1ll;
static uint16_t via_t1lh;
static uint16_t via_t1pb7; /* timer 1 controlled version of pb7 */
static uint16_t via_t2on;  /* is timer 2 on? */
static uint16_t via_t2int; /* are timer 2 interrupts allowed? */
static uint16_t via_t2c;
static uint16_t via_t2ll;
static uint16_t via_sr;
static uint16_t via_srb;   /* number of bits shifted so far */
static uint16_t via_src;   /* shift counter */
static uint16_t via_srclk;
static uint16_t via_acr;
static uint16_t via_pcr;
static uint16_t via_ifr;
static uint16_t via_ier;
static uint16_t via_ca2;
static uint16_t via_cb2h;  /* basic handshake version of cb2 */
static uint16_t via_cb2s;  /* version of cb2 controlled by the shift register */

/* analog devices */

static uint16_t alg_rsh;  /* zero ref sample and hold */
static uint16_t alg_xsh;  /* x sample and hold */
static uint16_t alg_ysh;  /* y sample and hold */
static uint16_t alg_zsh;  /* z sample and hold */
uint16_t alg_jch0;		  /* joystick direction channel 0 */
uint16_t alg_jch1;		  /* joystick direction channel 1 */
uint16_t alg_jch2;		  /* joystick direction channel 2 */
uint16_t alg_jch3;		  /* joystick direction channel 3 */
static uint16_t alg_jsh;  /* joystick sample and hold */

static uint16_t alg_compare;

static int32_t alg_dx;     /* delta x */
static int32_t alg_dy;     /* delta y */
static int32_t alg_curr_x; /* current x position */
static int32_t alg_curr_y; /* current y position */

enum {
	VECTREX_PDECAY	= PDECAY_INIT,      /* phosphor decay rate */

	/* number of 6809 cycles before a frame redraw */

	FCYCLES_INIT    = VECTREX_MHZ / VECTREX_PDECAY,

	/* max number of possible vectors that maybe on the screen at one time.
	 * one only needs VECTREX_MHZ / VECTREX_PDECAY but we need to also store
	 * deleted vectors in a single table
	 */

	VECTOR_CNT = VECTOR_CNT_INIT,//VECTREX_MHZ / VECTREX_PDECAY,

	VECTOR_HASH = VECTOR_CNT_INIT//65521
};

static uint16_t alg_vectoring; /* are we drawing a vector right now? */
static int32_t alg_vector_x0;
static int32_t alg_vector_y0;
static int32_t alg_vector_x1;
static int32_t alg_vector_y1;
static int32_t alg_vector_dx;
static int32_t alg_vector_dy;
static uint8_t alg_vector_color;

int32_t vector_draw_cnt;
int32_t vector_erse_cnt;
static vector_t *vectors_set;
//static vector_t vectors_set[2 * VECTOR_CNT];
vector_t *vectors_draw;
vector_t *vectors_erse;

static int16_t *vector_hash;

static int32_t fcycles;

/* update the snd chips internal registers when via_ora/via_orb changes */

static  void snd_update (void){
	switch (via_orb & 0x18) {
	case 0x00:
		/* the sound chip is disabled */
		break;
	case 0x08:
		/* the sound chip is sending data */
		break;
	case 0x10:
		/* the sound chip is recieving data */

		if (snd_select != 14) {
			snd_regs[snd_select] = via_ora;
			e8910_write(snd_select, via_ora);
		}

		break;
	case 0x18:
		/* the sound chip is latching an address */

		if ((via_ora & 0xf0) == 0x00) {
			snd_select = via_ora & 0x0f;
		}

		break;
	}
}

/* update the various analog values when orb is written. */

static  void alg_update (void)
{
	switch (via_orb & 0x06) {
	case 0x00:
		alg_jsh = alg_jch0;

		if ((via_orb & 0x01) == 0x00) {
			/* demultiplexor is on */
			alg_ysh = alg_xsh;
		}

		break;
	case 0x02:
		alg_jsh = alg_jch1;

		if ((via_orb & 0x01) == 0x00) {
			/* demultiplexor is on */
			alg_rsh = alg_xsh;
		}

		break;
	case 0x04:
		alg_jsh = alg_jch2;

		if ((via_orb & 0x01) == 0x00) {
			/* demultiplexor is on */

			if (alg_xsh > 0x80) {
				alg_zsh = alg_xsh - 0x80;
			} else {
				alg_zsh = 0;
			}
		}

		break;
	case 0x06:
		/* sound output line */
		alg_jsh = alg_jch3;
		break;
	}

	/* compare the current joystick direction with a reference */

	if (alg_jsh > alg_xsh) {
		alg_compare = 0x20;
	} else {
		alg_compare = 0;
	}

	/* compute the new "deltas" */

	alg_dx = (int32_t) alg_xsh - (int32_t) alg_rsh;
	alg_dy = (int32_t) alg_rsh - (int32_t) alg_ysh;
}

/* update IRQ and bit-7 of the ifr register after making an adjustment to
 * ifr.
 */

static  void int_update (void)
{
	if ((via_ifr & 0x7f) & (via_ier & 0x7f)) {
		via_ifr |= 0x80;
	} else {
		via_ifr &= 0x7f;
	}
}

uint8_t read8 (uint16_t address)
{
	uint8_t data;

	if ((address & 0xe000) == 0xe000) {
		/* rom */

		data = rom[address & 0x1fff];
	} else if ((address & 0xe000) == 0xc000) {
		if (address & 0x800) {
			/* ram */

			data = ram[address & 0x3ff];
		} else if (address & 0x1000) {
			/* io */

			switch (address & 0xf) {
			case 0x0:
				/* compare signal is an input so the value does not come from
				 * via_orb.
				 */

				if (via_acr & 0x80) {
					/* timer 1 has control of bit 7 */

					data = (uint8_t) ((via_orb & 0x5f) | via_t1pb7 | alg_compare);
				} else {
					/* bit 7 is being driven by via_orb */

					data = (uint8_t) ((via_orb & 0xdf) | alg_compare);
				}

				break;
			case 0x1:
				/* register 1 also performs handshakes if necessary */

				if ((via_pcr & 0x0e) == 0x08) {
					/* if ca2 is in pulse mode or handshake mode, then it
					 * goes low whenever ira is read.
					 */

					via_ca2 = 0;
				}

				/* fall through */

			case 0xf:
				if ((via_orb & 0x18) == 0x08) {
					/* the snd chip is driving port a */

					data = (uint8_t) snd_regs[snd_select];
				} else {
					data = (uint8_t) via_ora;
				}

				break;
			case 0x2:
				data = (uint8_t) via_ddrb;
				break;
			case 0x3:
				data = (uint8_t) via_ddra;
				break;
			case 0x4:
				/* T1 low order counter */

				data = (uint8_t) via_t1c;
				via_ifr &= 0xbf; /* remove timer 1 interrupt flag */

				via_t1on = 0; /* timer 1 is stopped */
				via_t1int = 0;
				via_t1pb7 = 0x80;

				int_update ();

				break;
			case 0x5:
				/* T1 high order counter */

				data = (uint8_t) (via_t1c >> 8);

				break;
			case 0x6:
				/* T1 low order latch */

				data = (uint8_t) via_t1ll;
				break;
			case 0x7:
				/* T1 high order latch */

				data = (uint8_t) via_t1lh;
				break;
			case 0x8:
				/* T2 low order counter */

				data = (uint8_t) via_t2c;
				via_ifr &= 0xdf; /* remove timer 2 interrupt flag */

				via_t2on = 0; /* timer 2 is stopped */
				via_t2int = 0;

				int_update ();

				break;
			case 0x9:
				/* T2 high order counter */

				data = (uint8_t) (via_t2c >> 8);
				break;
			case 0xa:
				data = (uint8_t) via_sr;
				via_ifr &= 0xfb; /* remove shift register interrupt flag */
				via_srb = 0;
				via_srclk = 1;

				int_update ();

				break;
			case 0xb:
				data = (uint8_t) via_acr;
				break;
			case 0xc:
				data = (uint8_t) via_pcr;
				break;
			case 0xd:
				/* interrupt flag register */

				data = (uint8_t) via_ifr;
				break;
			case 0xe:
				/* interrupt enable register */

				data = (uint8_t) (via_ier | 0x80);
				break;
			}
		}
	} else if (address < 0x8000) {
		/* cartridge */

		data = cart[address];
	} else {
		data = 0xff;
	}

	return data;
}

void write8 (uint16_t address, uint8_t data)
{
	if ((address & 0xe000) == 0xe000) {
		/* rom */
	} else if ((address & 0xe000) == 0xc000) {
		/* it is possible for both ram and io to be written at the same! */

		if (address & 0x800) {
			ram[address & 0x3ff] = data;
		}

		if (address & 0x1000) {
			switch (address & 0xf) {
			case 0x0:
				via_orb = data;

				snd_update ();

				alg_update ();

				if ((via_pcr & 0xe0) == 0x80) {
					/* if cb2 is in pulse mode or handshake mode, then it
					 * goes low whenever orb is written.
					 */

					via_cb2h = 0;
				}

				break;
			case 0x1:
				/* register 1 also performs handshakes if necessary */

				if ((via_pcr & 0x0e) == 0x08) {
					/* if ca2 is in pulse mode or handshake mode, then it
					 * goes low whenever ora is written.
					 */

					via_ca2 = 0;
				}

				/* fall through */

			case 0xf:
				via_ora = data;

				snd_update ();

				/* output of port a feeds directly into the dac which then
				 * feeds the x axis sample and hold.
				 */

				alg_xsh = data ^ 0x80;

				alg_update ();

				break;
			case 0x2:
				via_ddrb = data;
				break;
			case 0x3:
				via_ddra = data;
				break;
			case 0x4:
				/* T1 low order counter */

				via_t1ll = data;

				break;
			case 0x5:
				/* T1 high order counter */

				via_t1lh = data;
				via_t1c = (via_t1lh << 8) | via_t1ll;
				via_ifr &= 0xbf; /* remove timer 1 interrupt flag */

				via_t1on = 1; /* timer 1 starts running */
				via_t1int = 1;
				via_t1pb7 = 0;

				int_update ();

				break;
			case 0x6:
				/* T1 low order latch */

				via_t1ll = data;
				break;
			case 0x7:
				/* T1 high order latch */

				via_t1lh = data;
				break;
			case 0x8:
				/* T2 low order latch */

				via_t2ll = data;
				break;
			case 0x9:
				/* T2 high order latch/counter */

				via_t2c = (data << 8) | via_t2ll;
				via_ifr &= 0xdf;

				via_t2on = 1; /* timer 2 starts running */
				via_t2int = 1;

				int_update ();

				break;
			case 0xa:
				via_sr = data;
				via_ifr &= 0xfb; /* remove shift register interrupt flag */
				via_srb = 0;
				via_srclk = 1;

				int_update ();

				break;
			case 0xb:
				via_acr = data;
				break;
			case 0xc:
				via_pcr = data;


				if ((via_pcr & 0x0e) == 0x0c) {
					/* ca2 is outputting low */

					via_ca2 = 0;
				} else {
					/* ca2 is disabled or in pulse mode or is
					 * outputting high.
					 */

					via_ca2 = 1;
				}

				if ((via_pcr & 0xe0) == 0xc0) {
					/* cb2 is outputting low */

					via_cb2h = 0;
				} else {
					/* cb2 is disabled or is in pulse mode or is
					 * outputting high.
					 */

					via_cb2h = 1;
				}

				break;
			case 0xd:
				/* interrupt flag register */

				via_ifr &= ~(data & 0x7f);
				int_update ();

				break;
			case 0xe:
				/* interrupt enable register */

				if (data & 0x80) {
					via_ier |= data & 0x7f;
				} else {
					via_ier &= ~(data & 0x7f);
				}

				int_update ();

				break;
			}
		}
	} else if (address < 0x8000) {
		/* cartridge */
	}
}

void vecx_reset (void)
{
  static bool initVecFlag = 0;
  if (!initVecFlag){ 
    vectors_set = new vector_t[2 * VECTOR_CNT];
    vector_hash = new int16_t[VECTOR_CNT];
  }
  initVecFlag=1;
	uint16_t r;

	/* ram */

	for (r = 0; r < 1024; r++) {
		ram[r] = r & 0xff;
	}

	for (r = 0; r < 16; r++) {
		snd_regs[r] = 0;
		e8910_write(r, 0);
	}

	/* input buttons */

	snd_regs[14] = 0xff;
	e8910_write(14, 0xff);

	snd_select = 0;

	via_ora = 0;
	via_orb = 0;
	via_ddra = 0;
	via_ddrb = 0;
	via_t1on = 0;
	via_t1int = 0;
	via_t1c = 0;
	via_t1ll = 0;
	via_t1lh = 0;
	via_t1pb7 = 0x80;
	via_t2on = 0;
	via_t2int = 0;
	via_t2c = 0;
	via_t2ll = 0;
	via_sr = 0;
	via_srb = 8;
	via_src = 0;
	via_srclk = 0;
	via_acr = 0;
	via_pcr = 0;
	via_ifr = 0;
	via_ier = 0;
	via_ca2 = 1;
	via_cb2h = 1;
	via_cb2s = 0;

	alg_rsh = 128;
	alg_xsh = 128;
	alg_ysh = 128;
	alg_zsh = 0;
	alg_jch0 = 128;
	alg_jch1 = 128;
	alg_jch2 = 128;
	alg_jch3 = 128;
	alg_jsh = 128;

	alg_compare = 0; /* check this */

	alg_dx = 0;
	alg_dy = 0;
	alg_curr_x = ALG_MAX_X / 2;
	alg_curr_y = ALG_MAX_Y / 2;

	alg_vectoring = 0;

	vector_draw_cnt = 0;
	vector_erse_cnt = 0;
	vectors_draw = vectors_set;
	vectors_erse = vectors_set + VECTOR_CNT;

	fcycles = FCYCLES_INIT;

	e6809_read8 = read8;
	e6809_write8 = write8;

	e6809_reset ();
}

/* perform a single cycle worth of via emulation.
 * via_sstep0 is the first postion of the emulation.
 */

static  void via_sstep0 (void)
{
	uint16_t t2shift;

	if (via_t1on) {
		via_t1c--;

		if ((via_t1c & 0xffff) == 0xffff) {
			/* counter just rolled over */

			if (via_acr & 0x40) {
				/* continuous interrupt mode */

				via_ifr |= 0x40;
				int_update ();
				via_t1pb7 = 0x80 - via_t1pb7;

				/* reload counter */

				via_t1c = (via_t1lh << 8) | via_t1ll;
			} else {
				/* one shot mode */

				if (via_t1int) {
					via_ifr |= 0x40;
					int_update ();
					via_t1pb7 = 0x80;
					via_t1int = 0;
				}
			}
		}
	}

	if (via_t2on && (via_acr & 0x20) == 0x00) {
		via_t2c--;

		if ((via_t2c & 0xffff) == 0xffff) {
			/* one shot mode */

			if (via_t2int) {
				via_ifr |= 0x20;
				int_update ();
				via_t2int = 0;
			}
		}
	}

	/* shift counter */

	via_src--;

	if ((via_src & 0xff) == 0xff) {
		via_src = via_t2ll;

		if (via_srclk) {
			t2shift = 1;
			via_srclk = 0;
		} else {
			t2shift = 0;
			via_srclk = 1;
		}
	} else {
		t2shift = 0;
	}

	if (via_srb < 8) {
		switch (via_acr & 0x1c) {
		case 0x00:
			/* disabled */
			break;
		case 0x04:
			/* shift in under control of t2 */

			if (t2shift) {
				/* shifting in 0s since cb2 is always an output */

				via_sr <<= 1;
				via_srb++;
			}

			break;
		case 0x08:
			/* shift in under system clk control */

			via_sr <<= 1;
			via_srb++;

			break;
		case 0x0c:
			/* shift in under cb1 control */
			break;
		case 0x10:
			/* shift out under t2 control (free run) */

			if (t2shift) {
				via_cb2s = (via_sr >> 7) & 1;

				via_sr <<= 1;
				via_sr |= via_cb2s;
			}

			break;
		case 0x14:
			/* shift out under t2 control */

			if (t2shift) {
				via_cb2s = (via_sr >> 7) & 1;

				via_sr <<= 1;
				via_sr |= via_cb2s;
				via_srb++;
			}

			break;
		case 0x18:
			/* shift out under system clock control */

			via_cb2s = (via_sr >> 7) & 1;

			via_sr <<= 1;
			via_sr |= via_cb2s;
			via_srb++;

			break;
		case 0x1c:
			/* shift out under cb1 control */
			break;
		}

		if (via_srb == 8) {
			via_ifr |= 0x04;
			int_update ();
		}
	}
}

/* perform the second part of the via emulation */

static  void via_sstep1 (void)
{
	if ((via_pcr & 0x0e) == 0x0a) {
		/* if ca2 is in pulse mode, then make sure
		 * it gets restored to '1' after the pulse.
		 */

		via_ca2 = 1;
	}

	if ((via_pcr & 0xe0) == 0xa0) {
		/* if cb2 is in pulse mode, then make sure
		 * it gets restored to '1' after the pulse.
		 */

		via_cb2h = 1;
	}
}

static  void alg_addline (int32_t x0, int32_t y0, int32_t x1, int32_t y1, uint8_t color)
{
	uint32_t key;
	int32_t index;

	key = (uint32_t) x0;
	key = key * 31 + (uint32_t) y0;
	key = key * 31 + (uint32_t) x1;
	key = key * 31 + (uint32_t) y1;
	key %= VECTOR_HASH;

	/* first check if the line to be drawn is in the current draw list.
	 * if it is, then it is not added again.
	 */

	index = vector_hash[key];

	if (index >= 0 && index < vector_draw_cnt &&
		x0 == vectors_draw[index].x0 &&
		y0 == vectors_draw[index].y0 &&
		x1 == vectors_draw[index].x1 &&
		y1 == vectors_draw[index].y1) {
		vectors_draw[index].color = color;
	} else {
		/* missed on the draw list, now check if the line to be drawn is in
		 * the erase list ... if it is, "invalidate" it on the erase list.
		 */

		if (index >= 0 && index < vector_erse_cnt &&
			x0 == vectors_erse[index].x0 &&
			y0 == vectors_erse[index].y0 &&
			x1 == vectors_erse[index].x1 &&
			y1 == vectors_erse[index].y1) {
			vectors_erse[index].color = VECTREX_COLORS;
		}

		vectors_draw[vector_draw_cnt].x0 = x0;
		vectors_draw[vector_draw_cnt].y0 = y0;
		vectors_draw[vector_draw_cnt].x1 = x1;
		vectors_draw[vector_draw_cnt].y1 = y1;
		vectors_draw[vector_draw_cnt].color = color;
		vector_hash[key] = vector_draw_cnt;
		vector_draw_cnt++;
	}
}

/* perform a single cycle worth of analog emulation */

static  void alg_sstep (void)
{
	int32_t sig_dx, sig_dy;
	uint16_t sig_ramp;
	uint16_t sig_blank;

	if ((via_acr & 0x10) == 0x10) {
		sig_blank = via_cb2s;
	} else {
		sig_blank = via_cb2h;
	}

	if (via_ca2 == 0) {
		/* need to force the current point to the 'orgin' so just
		 * calculate distance to origin and use that as dx,dy.
		 */

		sig_dx = ALG_MAX_X / 2 - alg_curr_x;
		sig_dy = ALG_MAX_Y / 2 - alg_curr_y;
	} else {
		if (via_acr & 0x80) {
			sig_ramp = via_t1pb7;
		} else {
			sig_ramp = via_orb & 0x80;
		}

		if (sig_ramp == 0) {
			sig_dx = alg_dx;
			sig_dy = alg_dy;
		} else {
			sig_dx = 0;
			sig_dy = 0;
		}
	}

	if (alg_vectoring == 0) {
		if (sig_blank == 1 &&
			alg_curr_x >= 0 && alg_curr_x < ALG_MAX_X &&
			alg_curr_y >= 0 && alg_curr_y < ALG_MAX_Y) {

			/* start a new vector */

			alg_vectoring = 1;
			alg_vector_x0 = alg_curr_x;
			alg_vector_y0 = alg_curr_y;
			alg_vector_x1 = alg_curr_x;
			alg_vector_y1 = alg_curr_y;
			alg_vector_dx = sig_dx;
			alg_vector_dy = sig_dy;
			alg_vector_color = (uint8_t) alg_zsh;
		}
	} else {
		/* already drawing a vector ... check if we need to turn it off */

		if (sig_blank == 0) {
			/* blank just went on, vectoring turns off, and we've got a
			 * new line.
			 */

			alg_vectoring = 0;

			alg_addline (alg_vector_x0, alg_vector_y0,
						 alg_vector_x1, alg_vector_y1,
						 alg_vector_color);
		} else if (sig_dx != alg_vector_dx ||
				   sig_dy != alg_vector_dy ||
				   (uint8_t) alg_zsh != alg_vector_color) {

			/* the parameters of the vectoring processing has changed.
			 * so end the current line.
			 */

			alg_addline (alg_vector_x0, alg_vector_y0,
						 alg_vector_x1, alg_vector_y1,
						 alg_vector_color);

			/* we continue vectoring with a new set of parameters if the
			 * current point is not out of limits.
			 */

			if (alg_curr_x >= 0 && alg_curr_x < ALG_MAX_X &&
				alg_curr_y >= 0 && alg_curr_y < ALG_MAX_Y) {
				alg_vector_x0 = alg_curr_x;
				alg_vector_y0 = alg_curr_y;
				alg_vector_x1 = alg_curr_x;
				alg_vector_y1 = alg_curr_y;
				alg_vector_dx = sig_dx;
				alg_vector_dy = sig_dy;
				alg_vector_color = (uint8_t) alg_zsh;
			} else {
				alg_vectoring = 0;
			}
		}
	}

	alg_curr_x += sig_dx;
	alg_curr_y += sig_dy;

	if (alg_vectoring == 1 &&
		alg_curr_x >= 0 && alg_curr_x < ALG_MAX_X &&
		alg_curr_y >= 0 && alg_curr_y < ALG_MAX_Y) {

		/* we're vectoring ... current point is still within limits so
		 * extend the current vector.
		 */

		alg_vector_x1 = alg_curr_x;
		alg_vector_y1 = alg_curr_y;
	}
}

void vecx_emu (int32_t cycles)
{
	uint16_t c, icycles;

	while (cycles > 0) {
		icycles = e6809_sstep (via_ifr & 0x80, 0);

		for (c = 0; c < icycles; c++) {
			via_sstep0 ();
			alg_sstep ();
			via_sstep1 ();
		}

		cycles -= (int32_t) icycles;

		fcycles -= (int32_t) icycles;

		if (fcycles < 0) {
			vector_t *tmp;

			fcycles += FCYCLES_INIT;
			osint_render ();

			/* everything that was drawn during this pass now now enters
			 * the erase list for the next pass.
			 */

			vector_erse_cnt = vector_draw_cnt;
			vector_draw_cnt = 0;

			tmp = vectors_erse;
			vectors_erse = vectors_draw;
			vectors_draw = tmp;
		}
	}
}
