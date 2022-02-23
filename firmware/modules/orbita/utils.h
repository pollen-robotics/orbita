#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>

void toggle_status_led();
void status_led(uint8_t state);

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