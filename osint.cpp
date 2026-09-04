#include <math.h>
#include "gbConfig.h"

#include "osint.h"
#include "vecx.h"

#include "dataFlash/gbrom.h"
#include "dataFlash/gbbios.h"

#include <Arduino.h>

 static int screenx;
 static int screeny;
 static int32_t inv_scl_factor; // Заменили scl_factor на множитель
 static int offx;
 static int offy; 

// Изменился размер для кольцевого буфера
extern uint8_t *soundBuffer;

void osint_render(){
 static int v, x0, x1, y0, y1;

#ifdef SOUND_ENABLE 
  if (soundOnFlag) e8910_callback();
#endif

 if (vector_draw_cnt<=0){
  return;
 }

 screenClearBuffer();

  for(v = 0; v < vector_draw_cnt; v++){   
   x0 = (vectors_draw[v].x0);
   y0 = (vectors_draw[v].y0);
   x1 = (vectors_draw[v].x1);
   y1 = (vectors_draw[v].y1);

    // Fixed-point умножение вместо медленного деления
    x0 = offx + ((x0 * inv_scl_factor) >> 16);
    y0 = offy + ((y0 * inv_scl_factor) >> 16);

  if ((x0==x1) && (y0==y1)){
    screenPixel(x0, y0);
    }
  else{
    x1 = offx + ((x1 * inv_scl_factor) >> 16);
    y1 = offy + ((y1 * inv_scl_factor) >> 16);  
    if ((x0==x1) && (y0==y1)){
      screenPixel(x0, y0);
    }
    else{
     screenLine (x0, y0, x1, y1);
    }
   }
  }

  screenDrawBuffer();
}

void resize(int width, int height)
{
  long sclx, scly, scl_factor;
  
	screenx = width;
	screeny = height;
 	sclx = ALG_MAX_X / width;
 	scly = ALG_MAX_Y / height;         
	scl_factor = sclx > scly ? sclx : scly;
  inv_scl_factor = (1 << 16) / scl_factor; // Предрасчет для fixed-point
  
  offx = (screenx - ALG_MAX_X / scl_factor) / 2;
  offy = (screeny - ALG_MAX_Y / scl_factor) / 2;
}

bool checkKeys(){
  static unsigned char readKeys;
  readKeys = getKeysLocal();

  if ((readKeys&PAD_LEFT) && (readKeys&PAD_RIGHT) && (readKeys&PAD_UP) && (readKeys&PAD_DOWN)){
    if (soundOnFlag)  e8910_done_sound();
    else e8910_init_sound();
    soundOnFlag=!soundOnFlag;
    while(getKeysLocal()) ESP.wdtFeed();
  };

  if ((readKeys&PAD_RGT) && (readKeys&PAD_LFT))
    return 1;

  if (readKeys&PAD_LFT) snd_regs[14] &= ~0x01; 
  else snd_regs[14]|= 0x01; 

  if (readKeys&PAD_RGT) snd_regs[14] &= ~0x02; 
  else snd_regs[14]|= 0x02; 

  if (readKeys&PAD_ACT) snd_regs[14] &= ~0x04; 
  else snd_regs[14]|= 0x04; 

  if (readKeys&PAD_ESC) snd_regs[14] &= ~0x08; 
  else snd_regs[14]|= 0x08; 

  if (readKeys&PAD_LEFT) alg_jch0 = 0x00;
  else alg_jch0 = 0x80;

  if (readKeys&PAD_RIGHT) alg_jch0 = 0xff;
  
  if (readKeys&PAD_UP) alg_jch1 = 0xff;
  else alg_jch1 = 0x80;  

  if (readKeys&PAD_DOWN) alg_jch1 = 0x00;

  return 0;
}

void osint_emuloop(){
 vecx_reset();
 while(1){    
     ESP.wdtFeed();
     if (checkKeys()) break;  
     vecx_emu(VECTREX_MHZ/VECTREX_MHZ_DIV);

#ifdef SERIAL_DEBUG
     static uint32_t frameMillis;
     static uint32_t frameCounter=0;
     frameCounter++;
     if ((millis()-frameMillis) > 1000){
       Serial.print ("FPS: "); Serial.println(frameCounter);
       frameMillis = millis();
       frameCounter = 0;
       Serial.print ("Free heap: "); Serial.println(ESP.getFreeHeap());}
#endif
	}
}

void mainEmulator(){
  resize(128,128);
  rom = gb_rom_bios;
  if (!flagLoadedRom) cart = gb_cart_empty;
  else cart = allocatedRom;
  
#ifdef SOUND_ENABLE  
  e8910_init_sound();
#endif
  
  osint_emuloop();

#ifdef SOUND_ENABLE    
  e8910_done_sound();
#endif
}
