/** @file convolution.h*/

#ifndef CONVOLUTION_H
#define CONVOLUTION_H

#ifdef __cplusplus
extern "C" {
#endif
#include <stdbool.h>

/** Horizontal 1D convolution*/
bool convH(float* in, float* out, int dataSizeX, int dataSizeY, const float* kernelX, int kSizeX);

/** Vertical 1D convolution*/
bool convV(float* in, float* out, int dataSizeX, int dataSizeY, const float* kernelY, int kSizeY);

/** composite 2D convolution
*   @param in float* image
*   @return big if true
*/
bool convolve2DSeparable(float* in, float* out, int dataSizeX, int dataSizeY, const float* kernelX,
    int kSizeX, const float* kernelY, int kSizeY);

/** 8bit version composite 2D convolution
*   @param in uint8* image
*   @return big if true
*/
bool convolve2DSeparable8(unsigned char* in, unsigned char* out, int dataSizeX, int dataSizeY, 
                         float* kernelX, int kSizeX, float* kernelY, int kSizeY);

#ifdef __cplusplus
}
#endif

#endif
