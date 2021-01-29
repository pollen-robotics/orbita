#ifndef UTILS_H
#define UTILS_H

#include "luos.h"

void Orbita_FlashReadLuosMemoryInfo(uint32_t addr, uint16_t data_size, uint8_t *data);
void Orbita_FlashWriteLuosMemoryInfo(uint32_t addr, uint16_t data_size, uint8_t data[]);


 #define min(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })

 #define max(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })

#define clip(a, l, u) min(max(a, l), u)


#endif