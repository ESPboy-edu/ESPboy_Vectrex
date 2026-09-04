#ifndef _GB_CONFIG_H
 #define _GB_CONFIG_H

//#define SERIAL_DEBUG 
#define MAX_FILE_SIZE 9000
#define SOUND_BUFFER_SIZE 1024 // Должно быть степенью двойки!
#define FORE_GROUND_COLOR 0xFFE0
#define VECTOR_CNT_INIT  512 // Оптимизировано под степень двойки
#define PDECAY_INIT 23
#define VECTREX_MHZ_DIV  120

//#define SOUND_ENABLE  //TODO: this version needs improving
#define SOUND_FREQ   12000
#define SOUND_SAMPLE 3300
#define SOUNDPIN D3

#endif
