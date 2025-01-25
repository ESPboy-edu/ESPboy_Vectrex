#ifndef __VECX_H
#define __VECX_H

#include <Arduino.h>

enum {
	VECTREX_MHZ		= 1500000, /* speed of the vectrex being emulated */
	VECTREX_COLORS  = 1,     /* number of possible colors ... grayscale */

	ALG_MAX_X		= 33000,
	ALG_MAX_Y		= 41000
};

typedef struct vector_type {
	int32_t x0, y0; /* start coordinate */
	int32_t x1, y1; /* end coordinate */

	/* color [0, VECTREX_COLORS - 1], if color = VECTREX_COLORS, then this is
	 * an invalid entry and must be ignored.
	 */
	uint8_t color;
} vector_t;

extern uint8_t *rom;
extern uint8_t *cart;

extern uint16_t alg_jch0;
extern uint16_t alg_jch1;
extern uint16_t alg_jch2;
extern uint16_t alg_jch3;

extern int32_t vector_draw_cnt;
extern int32_t vector_erse_cnt;
extern vector_t *vectors_draw;
extern vector_t *vectors_erse;

void vecx_reset (void);
void vecx_emu (int32_t cycles);

#endif
