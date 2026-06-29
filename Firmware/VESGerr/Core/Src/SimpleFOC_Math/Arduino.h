#pragma once
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

// SimpleFOC's _atan2 function expects Arduino's macro versions of min() and max()
#ifndef min
  #define min(a,b) ((a)<(b)?(a):(b))
#endif

#ifndef max
  #define max(a,b) ((a)>(b)?(a):(b))
#endif
