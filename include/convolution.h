#ifndef CONVOLUTION_H
#define CONVOLUTION_H

#ifdef __cplusplus
extern "C" {
#endif
#include <stdbool.h>

bool convH(float* in, float* out, int dataSizeX, int dataSizeY, const float* kernelX, int kSizeX);
bool convV(float* in, float* out, int dataSizeX, int dataSizeY, const float* kernelY, int kSizeY);
bool convolve2DSeparable(float* in, float* out, int dataSizeX, int dataSizeY, const float* kernelX,
    int kSizeX, const float* kernelY, int kSizeY);
bool convolve2DSeparable8(unsigned char* in, unsigned char* out, int dataSizeX, int dataSizeY, 
                         float* kernelX, int kSizeX, float* kernelY, int kSizeY);

#ifdef __cplusplus
}
#endif

#endif
