/*
Port of jhawthorn vecx (https://github.com/jhawthorn/vecx)
to ESPboy www.espboy.com - RomanS 2025 espboy.edu@gmail.com
upload *.vec files to LittleFS ESP8266 internal flash drive (if no, then will be only one BIOS game available)
LFT+RGT keys to back to menu
UP+DOWN+LEFT+RIGHT to switch ON/OFF sounds (it plays faster without sounds)
*/

#include <sigma_delta.h>

#include "gbConfig.h"
#include "osint.h"

#include "lib/ESPboyInit.h"
#include "lib/ESPboyInit.cpp"
#include "lib/ESPboyMenuGUI.h"
#include "lib/ESPboyMenuGUI.cpp"
//#include "lib/ESPboyTerminalGUI.h"
//#include "lib/ESPboyTerminalGUI.cpp"
//#include "lib/ESPboyOTA2.h"
//#include "lib/ESPboyOTA2.cpp"
#include "gbConfig.h"
#include <LittleFS.h>
#include "nbSPI.h"

#define MAX_FILE_SIZE 10000
#define swp(a, b) {(a)^=(b); (b)^=(a); (a)^=(b);}


ESPboyInit myESPboy;
ESPboyMenuGUI menuGUI(&myESPboy);
//ESPboyTerminalGUI *terminalGUIobj = NULL;
//ESPboyOTA2 *OTA2obj = NULL;

extern uint8_t *soundBuffer;

uint8_t *screenBuffer;
uint16_t *lineBuf, *lineBuf1, *lineBuf2;
char *fileNamesBuf;
char **fileNamesList;
uint8_t *fileNum;
uint8_t *allocatedRom;
bool flagLoadedRom;
bool soundOnFlag = true;


unsigned char getKeysLocal() {
  return myESPboy.getKeys();
}


void screenDrawBuffer() { 
  static uint16_t positionBuffLine, addrBuffer;
  static uint8_t mask, getByte; 

#ifdef SERIAL_DEBUG
  static uint32_t timestmp, cnt=0;
  timestmp=micros();
#endif
  
  addrBuffer=0;
  for (uint8_t b=0; b<128/4; b++){
    positionBuffLine = 0;
    if (lineBuf == lineBuf1) lineBuf=lineBuf2;
    else lineBuf=lineBuf1;
    for (uint8_t i=0; i<4*128/8; i++){
        mask = 128;
        getByte = screenBuffer[addrBuffer++];
        for (uint8_t m=0; m<8; m++){
          if (getByte & mask) lineBuf[positionBuffLine] = 0xE0FF;
          else lineBuf[positionBuffLine] = 0;
          positionBuffLine++;
          mask = mask >> 1;
        }
      } 
    //myESPboy.tft.pushColors(lineBuf, 128*4);
    while(nbSPI_isBusy());
    nbSPI_writeBytes((uint8_t*)lineBuf, 128*8);
  }

#ifdef SERIAL_DEBUG
 if(cnt++>40) {cnt=0; Serial.print(F("PUSH DISPLAY MILLIS: "));  Serial.println(micros()-timestmp);};
#endif

}


void screenClearBuffer() {
  memset(screenBuffer, 0, 2048);
}


void screenPixel(int16_t x, int16_t y) {
  screenBuffer[(x + y*128)>>3] |=  (0x80 >> (x & 0x7));
}


void screenLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1) {
  static int32_t steep, inc, dx, dy, x, y, e; 
              
  steep = abs(y1 - y0) > abs(x1 - x0),
  inc=-1;

  if (steep) {
    swp(x0, y0);
    swp(x1, y1);}

  if (x0 > x1) {
    swp(x0, x1);
    swp(y0, y1);}

  if (y0 < y1) inc=1;

  dx = abs(x1 - x0);
  dy = abs(y1 - y0);
  y=y0; x=x0; e=0;

  for (x; x <= x1; x++) {
    if (steep) screenPixel(y, x);
    else screenPixel(x, y);
    if ((e + dy) << 1 < dx) e=e+dy;
    else {
      y+=inc;
      e=e+dy-dx;
    }
  }
}



void setup(){
#ifdef SERIAL_DEBUG
  Serial.begin(57600);
  Serial.println();
#endif
  
  myESPboy.begin("ESPboy Vectrex");

  /*
    //Check OTA2
  if (myESPboy.getKeys()&PAD_ACT || myESPboy.getKeys()&PAD_ESC) { 
     terminalGUIobj = new ESPboyTerminalGUI(&myESPboy.tft, &myESPboy.mcp);
     OTA2obj = new ESPboyOTA2(terminalGUIobj);
  }
  */
  
  screenBuffer = (uint8_t *)malloc(2048);
  memset (screenBuffer, 0 , 2048);

#ifdef SERIAL_DEBUG
  Serial.print(F("START  "));
  Serial.println(ESP.getFreeHeap());
#endif

  lineBuf1 = (uint16_t *)malloc (128*4*2);
  lineBuf2 = (uint16_t *)malloc (128*4*2);

#ifdef SERIAL_DEBUG
  Serial.print(F("Line buf alloc  "));
  Serial.println(ESP.getFreeHeap());
#endif
  
  ram = (unsigned char *)malloc (1024);

#ifdef SERIAL_DEBUG
  Serial.print(F("RAM alloc  "));
  Serial.println(ESP.getFreeHeap());
#endif


  soundBuffer = (uint8_t *)malloc (SOUND_SAMPLE+1);
#ifdef SERIAL_DEBUG
  Serial.print(F("Sound buf alloc  "));
  Serial.println(ESP.getFreeHeap());
#endif

myESPboy.tft.setWindow(0, 0, 127, 127); 
}



void loop(){
  uint8_t fileCnt=0, fileSCnt=0;
  flagLoadedRom = 0;
  
  if(LittleFS.begin()){
    Dir dir = LittleFS.openDir("/");
    while (dir.next()){
      if(strstr(dir.fileName().c_str(),".vec") && dir.fileSize()<MAX_FILE_SIZE){
        fileCnt++;}}
        
   if (fileCnt){
     fileNamesBuf = (char *)malloc (20*fileCnt);
     fileNamesList = (char **)malloc ((fileCnt+1) * sizeof(char*));
     fileNum = (uint8_t *)malloc (fileCnt);
     dir.rewind();  
     fileCnt=0;
     while (dir.next()){
       if(strstr(dir.fileName().c_str(),".vec") && dir.fileSize()<MAX_FILE_SIZE){
#ifdef SERIAL_DEBUG
         Serial.print(dir.fileName().c_str()); Serial.print("      "); Serial.println(dir.fileSize());
#endif
         strncpy(&fileNamesBuf[20*fileCnt],dir.fileName().c_str(),19);
         fileNamesList[fileCnt] = &fileNamesBuf[20*fileCnt];
         fileNum[fileCnt] = fileSCnt;
         fileCnt++;
       }
       fileSCnt++;
     }
     fileNamesList[fileCnt+1] = NULL;
     flagLoadedRom = 1;
     
     uint8_t selectedFile = menuGUI.menuInit((const char **)fileNamesList, TFT_YELLOW, TFT_BLUE, TFT_BLUE);  
     myESPboy.tft.fillScreen(TFT_BLACK);
     dir.rewind();
     while(fileNum[selectedFile]--) dir.next();
     allocatedRom = (uint8_t *)malloc(dir.fileSize());
     if (allocatedRom == NULL){
        flagLoadedRom = 0;
        myESPboy.tft.fillScreen(TFT_BLACK);
        myESPboy.tft.drawString("Loading error", 24,50);
        delay(3000);
        ESP.reset();
     }
     else{
       File f = dir.openFile("r");
       f.readBytes((char *)allocatedRom, dir.fileSize());
       f.close();}

#ifdef SERIAL_DEBUG
  Serial.print(F("ROM alloc  "));
  Serial.println(ESP.getFreeHeap());
#endif

     LittleFS.end();
     free(fileNamesBuf);
     free(fileNamesList);
     free(fileNum);

   }
  }

#ifdef SERIAL_DEBUG
  Serial.print(F("Free files list  "));
  Serial.println(ESP.getFreeHeap());
#endif

  mainEmulator();
  free(allocatedRom);

#ifdef SERIAL_DEBUG
  Serial.print(F("Free ROM  "));
  Serial.println(ESP.getFreeHeap());
#endif

  myESPboy.tft.fillScreen(TFT_BLACK);

}
