#ifndef __OSINT_H
#define __OSINT_H

#define PAD_LEFT        0x01
#define PAD_UP          0x02
#define PAD_DOWN        0x04
#define PAD_RIGHT       0x08
#define PAD_ACT         0x10
#define PAD_ESC         0x20
#define PAD_LFT         0x40
#define PAD_RGT         0x80
#define PAD_ANY         0xff

extern void e8910_callback();
extern void e8910_init_sound();
extern void e8910_done_sound();

extern bool soundOnFlag;
extern uint8_t snd_regs[16];
extern bool flagLoadedRom;
extern uint8_t *allocatedRom;
extern unsigned char getKeysLocal();
extern void screenClearBuffer();
extern void screenDrawBuffer();
extern void screenLine(int16_t, int16_t, int16_t, int16_t);
extern void screenPixel(int16_t, int16_t);

extern unsigned char *rom;
extern unsigned char *cart;
extern unsigned char *ram;

void osint_render (void);
void mainEmulator(void);

#endif
