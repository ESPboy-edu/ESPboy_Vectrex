#pragma GCC optimize ("O2")

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sigma_delta.h>
#include <Arduino.h>
#include "gbConfig.h"

#ifndef SOUND_ENABLE 
  #undef SOUND_SAMPLE
  #define SOUND_SAMPLE 1 
#endif

// Кольцевой буфер (размер должен быть степенью двойки)
#define RING_BUFFER_SIZE SOUND_BUFFER_SIZE
#define RING_BUFFER_MASK (RING_BUFFER_SIZE - 1)

volatile uint16_t sound_head = 0;
volatile uint16_t sound_tail = 0;
uint8_t *soundBuffer;

extern uint8_t snd_regs[16];

/***************************************************************************

  ay8910.c

  Emulation of the AY-3-8910 / YM2149 sound chip.
  Based on various code snippets by Ville Hallik, Michael Cuddy,
  Tatsuyuki Satoh, Fabrice Frances, Nicola Salmoria.

***************************************************************************/

#define MAX_OUTPUT 0xff
#define STEP3 1
#define STEP2 length_
#define STEP  2

typedef int32_t       INT32;
typedef unsigned int  UINT32;
typedef char          INT8;
typedef unsigned char uint8_t_t;

struct AY8910 {
	int index;
	int ready;
#ifdef SOUND_ENABLE 
	uint8_t *Regs = snd_regs;
#else
  unsigned *Regs = NULL;
#endif
	INT32 lastEnable;
	INT32 PeriodA,PeriodB,PeriodC,PeriodN,PeriodE;
	INT32 CountA,CountB,CountC,CountN,CountE;
	UINT32 VolA,VolB,VolC,VolE;
	uint8_t_t EnvelopeA,EnvelopeB,EnvelopeC;
	uint8_t_t OutputA,OutputB,OutputC,OutputN;
	INT8 CountEnv;
	uint8_t_t Hold,Alternate,Attack,Holding;
	INT32 RNG;
	unsigned int VolTable[32];

} PSG;

#define AY_AFINE	(0)
#define AY_ACOARSE	(1)
#define AY_BFINE	(2)
#define AY_BCOARSE	(3)
#define AY_CFINE	(4)
#define AY_CCOARSE	(5)
#define AY_NOISEPER	(6)
#define AY_ENABLE	(7)
#define AY_AVOL		(8)
#define AY_BVOL		(9)
#define AY_CVOL		(10)
#define AY_EFINE	(11)
#define AY_ECOARSE	(12)
#define AY_ESHAPE	(13)
#define AY_PORTA	(14)
#define AY_PORTB	(15)

// Чтение из кольцевого буфера
void ICACHE_RAM_ATTR sound_ISR(){    
   uint8_t sample = 0;
   if (sound_head != sound_tail) {
       sample = soundBuffer[sound_tail];
       sound_tail = (sound_tail + 1) & RING_BUFFER_MASK; 
   }
   sigmaDeltaWrite(0, sample);
}

void e8910_write(int r, int v)
{
	int old;
    if (PSG.Regs == NULL) return;
	PSG.Regs[r] = v;

	switch( r )
	{
	case AY_AFINE:
	case AY_ACOARSE:
		PSG.Regs[AY_ACOARSE] &= 0x0f;
		old = PSG.PeriodA;
		PSG.PeriodA = (PSG.Regs[AY_AFINE] + 256 * PSG.Regs[AY_ACOARSE]) * STEP3;
		if (PSG.PeriodA == 0) PSG.PeriodA = STEP3;
		PSG.CountA += PSG.PeriodA - old;
		if (PSG.CountA <= 0) PSG.CountA = 1;
		break;
	case AY_BFINE:
	case AY_BCOARSE:
		PSG.Regs[AY_BCOARSE] &= 0x0f;
		old = PSG.PeriodB;
		PSG.PeriodB = (PSG.Regs[AY_BFINE] + 256 * PSG.Regs[AY_BCOARSE]) * STEP3;
		if (PSG.PeriodB == 0) PSG.PeriodB = STEP3;
		PSG.CountB += PSG.PeriodB - old;
		if (PSG.CountB <= 0) PSG.CountB = 1;
		break;
	case AY_CFINE:
	case AY_CCOARSE:
		PSG.Regs[AY_CCOARSE] &= 0x0f;
		old = PSG.PeriodC;
		PSG.PeriodC = (PSG.Regs[AY_CFINE] + 256 * PSG.Regs[AY_CCOARSE]) * STEP3;
		if (PSG.PeriodC == 0) PSG.PeriodC = STEP3;
		PSG.CountC += PSG.PeriodC - old;
		if (PSG.CountC <= 0) PSG.CountC = 1;
		break;
	case AY_NOISEPER:
		PSG.Regs[AY_NOISEPER] &= 0x1f;
		old = PSG.PeriodN;
		PSG.PeriodN = PSG.Regs[AY_NOISEPER] * STEP3;
		if (PSG.PeriodN == 0) PSG.PeriodN = STEP3;
		PSG.CountN += PSG.PeriodN - old;
		if (PSG.CountN <= 0) PSG.CountN = 1;
		break;
	case AY_ENABLE:
		PSG.lastEnable = PSG.Regs[AY_ENABLE];
		break;
	case AY_AVOL:
		PSG.Regs[AY_AVOL] &= 0x1f;
		PSG.EnvelopeA = PSG.Regs[AY_AVOL] & 0x10;
		PSG.VolA = PSG.EnvelopeA ? PSG.VolE : PSG.VolTable[PSG.Regs[AY_AVOL] ? PSG.Regs[AY_AVOL]*2+1 : 0];
		break;
	case AY_BVOL:
		PSG.Regs[AY_BVOL] &= 0x1f;
		PSG.EnvelopeB = PSG.Regs[AY_BVOL] & 0x10;
		PSG.VolB = PSG.EnvelopeB ? PSG.VolE : PSG.VolTable[PSG.Regs[AY_BVOL] ? PSG.Regs[AY_BVOL]*2+1 : 0];
		break;
	case AY_CVOL:
		PSG.Regs[AY_CVOL] &= 0x1f;
		PSG.EnvelopeC = PSG.Regs[AY_CVOL] & 0x10;
		PSG.VolC = PSG.EnvelopeC ? PSG.VolE : PSG.VolTable[PSG.Regs[AY_CVOL] ? PSG.Regs[AY_CVOL]*2+1 : 0];
		break;
	case AY_EFINE:
	case AY_ECOARSE:
		old = PSG.PeriodE;
		PSG.PeriodE = ((PSG.Regs[AY_EFINE] + 256 * PSG.Regs[AY_ECOARSE])) * STEP3;
		if (PSG.PeriodE == 0) PSG.PeriodE = STEP3;
		PSG.CountE += PSG.PeriodE - old;
		if (PSG.CountE <= 0) PSG.CountE = 1;
		break;
	case AY_ESHAPE:
		PSG.Regs[AY_ESHAPE] &= 0x0f;
		PSG.Attack = (PSG.Regs[AY_ESHAPE] & 0x04) ? 0x1f : 0x00;
		if ((PSG.Regs[AY_ESHAPE] & 0x08) == 0)
		{
			PSG.Hold = 1;
			PSG.Alternate = PSG.Attack;
		}
		else
		{
			PSG.Hold = PSG.Regs[AY_ESHAPE] & 0x01;
			PSG.Alternate = PSG.Regs[AY_ESHAPE] & 0x02;
		}
		PSG.CountE = PSG.PeriodE;
		PSG.CountEnv = 0x1f;
		PSG.Holding = 0;
		PSG.VolE = PSG.VolTable[PSG.CountEnv ^ PSG.Attack];
		if (PSG.EnvelopeA) PSG.VolA = PSG.VolE;
		if (PSG.EnvelopeB) PSG.VolB = PSG.VolE;
		if (PSG.EnvelopeC) PSG.VolC = PSG.VolE;
		break;
	case AY_PORTA:
		break;
	case AY_PORTB:
		break;
	}
}

void e8910_callback(){
    // Считаем, сколько байт успело проиграться и освободиться
    uint16_t length_ = (sound_tail - sound_head - 1) & RING_BUFFER_MASK; 
    if (length_ > 800) length_ = 800;
    if (length_ == 0) return; 

    int outn;

    if (PSG.Regs[AY_ENABLE] & 0x01) {
		if (PSG.CountA <= STEP2) PSG.CountA += STEP2;
		PSG.OutputA = 1;
	} else if (PSG.Regs[AY_AVOL] == 0) {
		if (PSG.CountA <= STEP2) PSG.CountA += STEP2;
	}
	if (PSG.Regs[AY_ENABLE] & 0x02) {
		if (PSG.CountB <= STEP2) PSG.CountB += STEP2;
		PSG.OutputB = 1;
	} else if (PSG.Regs[AY_BVOL] == 0) {
		if (PSG.CountB <= STEP2) PSG.CountB += STEP2;
	}
	if (PSG.Regs[AY_ENABLE] & 0x04) {
		if (PSG.CountC <= STEP2) PSG.CountC += STEP2;
		PSG.OutputC = 1;
	} else if (PSG.Regs[AY_CVOL] == 0) {
		if (PSG.CountC <= STEP2) PSG.CountC += STEP2;
	}

	if ((PSG.Regs[AY_ENABLE] & 0x38) == 0x38)
		if (PSG.CountN <= STEP2) PSG.CountN += STEP2;

	outn = (PSG.OutputN | PSG.Regs[AY_ENABLE]);

	while (length_ > 0)
	{
        uint32_t vol;
        int left  = 2;
		uint32_t vola=0, volb=0, volc=0;

		do
		{
			int nextevent;
			if (PSG.CountN < left) nextevent = PSG.CountN;
			else nextevent = left;

			if (outn & 0x08)
			{
				if (PSG.OutputA) vola += PSG.CountA;
				PSG.CountA -= nextevent;
				while (PSG.CountA <= 0)
				{
					PSG.CountA += PSG.PeriodA;
					if (PSG.CountA > 0)
					{
						PSG.OutputA ^= 1;
						if (PSG.OutputA) vola += PSG.PeriodA;
						break;
					}
					PSG.CountA += PSG.PeriodA;
					vola += PSG.PeriodA;
				}
				if (PSG.OutputA) vola -= PSG.CountA;
			}
			else
			{
				PSG.CountA -= nextevent;
				while (PSG.CountA <= 0)
				{
					PSG.CountA += PSG.PeriodA;
					if (PSG.CountA > 0)
					{
						PSG.OutputA ^= 1;
						break;
					}
					PSG.CountA += PSG.PeriodA;
				}
			}

			if (outn & 0x10)
			{
				if (PSG.OutputB) volb += PSG.CountB;
				PSG.CountB -= nextevent;
				while (PSG.CountB <= 0)
				{
					PSG.CountB += PSG.PeriodB;
					if (PSG.CountB > 0)
					{
						PSG.OutputB ^= 1;
						if (PSG.OutputB) volb += PSG.PeriodB;
						break;
					}
					PSG.CountB += PSG.PeriodB;
					volb += PSG.PeriodB;
				}
				if (PSG.OutputB) volb -= PSG.CountB;
			}
			else
			{
				PSG.CountB -= nextevent;
				while (PSG.CountB <= 0)
				{
					PSG.CountB += PSG.PeriodB;
					if (PSG.CountB > 0)
					{
						PSG.OutputB ^= 1;
						break;
					}
					PSG.CountB += PSG.PeriodB;
				}
			}

			if (outn & 0x20)
			{
				if (PSG.OutputC) volc += PSG.CountC;
				PSG.CountC -= nextevent;
				while (PSG.CountC <= 0)
				{
					PSG.CountC += PSG.PeriodC;
					if (PSG.CountC > 0)
					{
						PSG.OutputC ^= 1;
						if (PSG.OutputC) volc += PSG.PeriodC;
						break;
					}
					PSG.CountC += PSG.PeriodC;
					volc += PSG.PeriodC;
				}
				if (PSG.OutputC) volc -= PSG.CountC;
			}
			else
			{
				PSG.CountC -= nextevent;
				while (PSG.CountC <= 0)
				{
					PSG.CountC += PSG.PeriodC;
					if (PSG.CountC > 0)
					{
						PSG.OutputC ^= 1;
						break;
					}
					PSG.CountC += PSG.PeriodC;
				}
			}

			PSG.CountN -= nextevent;
			if (PSG.CountN <= 0)
			{
				if ((PSG.RNG + 1) & 2)
				{
					PSG.OutputN = ~PSG.OutputN;
					outn = (PSG.OutputN | PSG.Regs[AY_ENABLE]);
				}
				if (PSG.RNG & 1) PSG.RNG ^= 0x24000; 
				PSG.RNG >>= 1;
				PSG.CountN += PSG.PeriodN;
			}

			left -= nextevent;
		} while (left > 0);

		if (PSG.Holding == 0)
		{
			PSG.CountE -= STEP;
			if (PSG.CountE <= 0)
			{
				do
				{
					PSG.CountEnv--;
					PSG.CountE += PSG.PeriodE;
				} while (PSG.CountE <= 0);

				if (PSG.CountEnv < 0)
				{
					if (PSG.Hold)
					{
						if (PSG.Alternate)
							PSG.Attack ^= 0x1f;
						PSG.Holding = 1;
						PSG.CountEnv = 0;
					}
					else
					{
						if (PSG.Alternate && (PSG.CountEnv & 0x20))
 							PSG.Attack ^= 0x1f;
						PSG.CountEnv &= 0x1f;
					}
				}

				PSG.VolE = PSG.VolTable[PSG.CountEnv ^ PSG.Attack];
				if (PSG.EnvelopeA) PSG.VolA = PSG.VolE;
				if (PSG.EnvelopeB) PSG.VolB = PSG.VolE;
				if (PSG.EnvelopeC) PSG.VolC = PSG.VolE;
			}
		}

        // Оптимизация деления через сдвиг
        vol = (vola * PSG.VolA + volb * PSG.VolB + volc * PSG.VolC) >> 1;
	    if (vol > 255) vol = 255;
	  
        // Запись в кольцевой буфер
        uint16_t next_head = (sound_head + 1) & RING_BUFFER_MASK;
        if (next_head != sound_tail) {
            soundBuffer[sound_head] = vol;
            sound_head = next_head;
        }

        length_--;
	}
}

static void e8910_build_mixer_table(){
	int i;
	double out;
	out = MAX_OUTPUT;
	for (i = 31;i > 0;i--)
	{
		PSG.VolTable[i] = (unsigned)(out + 0.5);	
		out /= 1.188502227;	
	}
	PSG.VolTable[0] = 0;
}

void e8910_init_sound(){
      memset (soundBuffer, 0, RING_BUFFER_SIZE);
      e8910_build_mixer_table();
      noInterrupts();
      sigmaDeltaSetup(0, SOUND_FREQ);
      sigmaDeltaAttachPin(SOUNDPIN);
      sigmaDeltaEnable();
      timer1_attachInterrupt(sound_ISR);
      timer1_enable(TIM_DIV1, TIM_EDGE, TIM_LOOP);
      timer1_write(80 * 1000000 / SOUND_FREQ);
      interrupts();
}

void e8910_done_sound(){
  noInterrupts();
  timer1_disable();
  interrupts();
  delay(10);
}
