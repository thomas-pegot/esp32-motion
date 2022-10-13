/** @file convolution.c
* Code extracted from : http://www.songho.ca/dsp/convolution/convolution.html
* And modified such as it perform a composition of 1D convolution horiz + vertical
* for separable filter
*/


#include "convolution.h"
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <stdlib.h> 


///////////////////////////////////////////////////////////////////////////////
// unsigned char (8-bit) version
///////////////////////////////////////////////////////////////////////////////
bool convolve2DSeparable8(unsigned char* in, unsigned char* out, int dataSizeX, int dataSizeY, 
                         float* kernelX, int kSizeX, float* kernelY, int kSizeY)
{
    int i, j, k, m, n;
    float *tmp, *sum;                               // intermediate data buffer
    unsigned char *inPtr, *outPtr;                  // working pointers
    float *tmpPtr, *tmpPtr2;                        // working pointers
    int kCenter, kOffset, endIndex;                 // kernel indice

    // check validity of params
    if(!in || !out || !kernelX || !kernelY) return false;
    if(dataSizeX <= 0 || kSizeX <= 0) return false;

    // allocate temp storage to keep intermediate result
    tmp = (float*)malloc(dataSizeX * dataSizeY * sizeof(float));
    if (!tmp) return false;  // memory allocation error

    // store accumulated sum
    sum = (float*)malloc(dataSizeX * sizeof(float));
    if (!sum)  return false;  // memory allocation error

    // covolve horizontal direction ///////////////////////

    // find center position of kernel (half of kernel size)
    kCenter = kSizeX >> 1;                          // center index of kernel array
    endIndex = dataSizeX - kCenter;                 // index for full kernel convolution

    // init working pointers
    inPtr = in;
    tmpPtr = tmp;                                   // store intermediate results from 1D horizontal convolution

    // start horizontal convolution (x-direction)
    for(i=0; i < dataSizeY; ++i)                    // number of rows
    {

        kOffset = 0;                                // starting index of partial kernel varies for each sample

        // COLUMN FROM index=0 TO index=kCenter-1
        for(j=0; j < kCenter; ++j)
        {
            *tmpPtr = 0;                            // init to 0 before accumulation

            for(k = kCenter + kOffset, m = 0; k >= 0; --k, ++m) // convolve with partial of kernel
            {
                *tmpPtr += *(inPtr + m) * kernelX[k];
            }
            ++tmpPtr;                               // next output
            ++kOffset;                              // increase starting index of kernel
        }

        // COLUMN FROM index=kCenter TO index=(dataSizeX-kCenter-1)
        for(j = kCenter; j < endIndex; ++j)
        {
            *tmpPtr = 0;                            // init to 0 before accumulate

            for(k = kSizeX-1, m = 0; k >= 0; --k, ++m)  // full kernel
            {
                *tmpPtr += *(inPtr + m) * kernelX[k];
            }
            ++inPtr;                                // next input
            ++tmpPtr;                               // next output
        }

        kOffset = 1;                                // ending index of partial kernel varies for each sample

        // COLUMN FROM index=(dataSizeX-kCenter) TO index=(dataSizeX-1)
        for(j = endIndex; j < dataSizeX; ++j)
        {
            *tmpPtr = 0;                            // init to 0 before accumulation

            for(k = kSizeX-1, m=0; k >= kOffset; --k, ++m)   // convolve with partial of kernel
            {
                *tmpPtr += *(inPtr + m) * kernelX[k];
            }
            ++inPtr;                                // next input
            ++tmpPtr;                               // next output
            ++kOffset;                              // increase ending index of partial kernel
        }

        inPtr += kCenter;                           // next row
    }
    // END OF HORIZONTAL CONVOLUTION //////////////////////

    // start vertical direction ///////////////////////////

    // find center position of kernel (half of kernel size)
    kCenter = kSizeY >> 1;                          // center index of vertical kernel
    endIndex = dataSizeY - kCenter;                 // index where full kernel convolution should stop

    // set working pointers
    tmpPtr = tmpPtr2 = tmp;
    outPtr = out;

    // clear out array before accumulation
    for(i = 0; i < dataSizeX; ++i)
        sum[i] = 0;

    // start to convolve vertical direction (y-direction)

    // ROW FROM index=0 TO index=(kCenter-1)
    kOffset = 0;                                    // starting index of partial kernel varies for each sample
    for(i=0; i < kCenter; ++i)
    {
        for(k = kCenter + kOffset; k >= 0; --k)     // convolve with partial kernel
        {
            for(j=0; j < dataSizeX; ++j)
            {
                sum[j] += *tmpPtr * kernelY[k];
                ++tmpPtr;
            }
        }

        for(n = 0; n < dataSizeX; ++n)              // convert and copy from sum to out
        {
            // covert negative to positive
            *outPtr = (unsigned char)((float)fabs(sum[n]) + 0.5f);
            sum[n] = 0;                             // reset to zero for next summing
            ++outPtr;                               // next element of output
        }

        tmpPtr = tmpPtr2;                           // reset input pointer
        ++kOffset;                                  // increase starting index of kernel
    }

    // ROW FROM index=kCenter TO index=(dataSizeY-kCenter-1)
    for(i = kCenter; i < endIndex; ++i)
    {
        for(k = kSizeY -1; k >= 0; --k)             // convolve with full kernel
        {
            for(j = 0; j < dataSizeX; ++j)
            {
                sum[j] += *tmpPtr * kernelY[k];
                ++tmpPtr;
            }
        }

        for(n = 0; n < dataSizeX; ++n)              // convert and copy from sum to out
        {
            // covert negative to positive
            *outPtr = (unsigned char)((float)fabs(sum[n]) + 0.5f);
            sum[n] = 0;                             // reset for next summing
            ++outPtr;                               // next output
        }

        // move to next row
        tmpPtr2 += dataSizeX;
        tmpPtr = tmpPtr2;
    }

    // ROW FROM index=(dataSizeY-kCenter) TO index=(dataSizeY-1)
    kOffset = 1;                                    // ending index of partial kernel varies for each sample
    for(i=endIndex; i < dataSizeY; ++i)
    {
        for(k = kSizeY-1; k >= kOffset; --k)        // convolve with partial kernel
        {
            for(j=0; j < dataSizeX; ++j)
            {
                sum[j] += *tmpPtr * kernelY[k];
                ++tmpPtr;
            }
        }

        for(n = 0; n < dataSizeX; ++n)              // convert and copy from sum to out
        {
            // covert negative to positive
            *outPtr = (unsigned char)((float)fabs(sum[n]) + 0.5f);
            sum[n] = 0;                             // reset for next summing
            ++outPtr;                               // next output
        }

        // move to next row
        tmpPtr2 += dataSizeX;
        tmpPtr = tmpPtr2;                           // next input
        ++kOffset;                                  // increase ending index of kernel
    }
    // END OF VERTICAL CONVOLUTION ////////////////////////

    // deallocate temp buffers
    free(tmp);
    free(sum);
    return true;
}

///////////////////////////////////////////////////////////////////////////////
// float version
///////////////////////////////////////////////////////////////////////////////
bool convolve2DSeparable(float* in, float* out, int dataSizeX, int dataSizeY, const float* kernelX,
    int kSizeX, const float* kernelY, int kSizeY)
{
    int i, j, k, m, n;
    float *tmp, *sum;                // intermediate data buffer
    float *inPtr, *outPtr;   // working pointers
    float *tmpPtr, *tmpPtr2;         // working pointers
    int kCenter, kOffset, endIndex;  // kernel indice

    // check validity of params
    if (!in || !out || !kernelX || !kernelY)
        return false;
    if (dataSizeX <= 0 || kSizeX <= 0)
        return false;

    // allocate temp storage to keep intermediate result
    //tmp = new float[dataSizeX * dataSizeY];
    tmp = (float*)malloc(dataSizeX * dataSizeY * sizeof(float));
    if (!tmp)
        return false;  // memory allocation error

    // store accumulated sum
    //sum = new float[dataSizeX];
    sum = (float*)malloc(dataSizeX * sizeof(float));
    if (!sum)
        return false;  // memory allocation error

    // covolve horizontal direction ///////////////////////

    // find center position of kernel (half of kernel size)
    kCenter = kSizeX >> 1;           // center index of kernel array
    endIndex = dataSizeX - kCenter;  // index for full kernel convolution

    // init working pointers
    inPtr = in;
    tmpPtr = tmp;  // store intermediate results from 1D horizontal convolution

    // start horizontal convolution (x-direction)
    for (i = 0; i < dataSizeY; ++i)  // number of rows
    {
        kOffset = 0;  // starting index of partial kernel varies for each sample

        // COLUMN FROM index=0 TO index=kCenter-1
        for (j = 0; j < kCenter; ++j)
        {
            *tmpPtr = 0;  // init to 0 before accumulation

            for (k = kCenter + kOffset, m = 0; k >= 0; --k, ++m)  // convolve with partial of kernel
            {
                *tmpPtr += *(inPtr + m) * kernelX[k];
            }
            ++tmpPtr;   // next output
            ++kOffset;  // increase starting index of kernel
        }

        // COLUMN FROM index=kCenter TO index=(dataSizeX-kCenter-1)
        for (j = kCenter; j < endIndex; ++j)
        {
            *tmpPtr = 0;  // init to 0 before accumulate

            for (k = kSizeX - 1, m = 0; k >= 0; --k, ++m)  // full kernel
            {
                *tmpPtr += *(inPtr + m) * kernelX[k];
            }
            ++inPtr;   // next input
            ++tmpPtr;  // next output
        }

        kOffset = 1;  // ending index of partial kernel varies for each sample

        // COLUMN FROM index=(dataSizeX-kCenter) TO index=(dataSizeX-1)
        for (j = endIndex; j < dataSizeX; ++j)
        {
            *tmpPtr = 0;  // init to 0 before accumulation

            for (k = kSizeX - 1, m = 0; k >= kOffset; --k, ++m)  // convolve with partial of kernel
            {
                *tmpPtr += *(inPtr + m) * kernelX[k];
            }
            ++inPtr;    // next input
            ++tmpPtr;   // next output
            ++kOffset;  // increase ending index of partial kernel
        }

        inPtr += kCenter;  // next row
    }
    // END OF HORIZONTAL CONVOLUTION //////////////////////

    // start vertical direction ///////////////////////////

    // find center position of kernel (half of kernel size)
    kCenter = kSizeY >> 1;           // center index of vertical kernel
    endIndex = dataSizeY - kCenter;  // index where full kernel convolution should stop

    // set working pointers
    tmpPtr = tmpPtr2 = tmp;
    outPtr = out;

    // clear out array before accumulation
    for (i = 0; i < dataSizeX; ++i)
        sum[i] = 0;

    // start to convolve vertical direction (y-direction)

    // ROW FROM index=0 TO index=(kCenter-1)
    kOffset = 0;  // starting index of partial kernel varies for each sample
    for (i = 0; i < kCenter; ++i)
    {
        for (k = kCenter + kOffset; k >= 0; --k)  // convolve with partial kernel
        {
            for (j = 0; j < dataSizeX; ++j)
            {
                sum[j] += *tmpPtr * kernelY[k];
                ++tmpPtr;
            }
        }

        for (n = 0; n < dataSizeX; ++n)  // convert and copy from sum to out
        {
            // covert negative to positive
            *outPtr = (float)((float)fabs(sum[n]) + 0.5f);
            sum[n] = 0;  // reset to zero for next summing
            ++outPtr;    // next element of output
        }

        tmpPtr = tmpPtr2;  // reset input pointer
        ++kOffset;         // increase starting index of kernel
    }

    // ROW FROM index=kCenter TO index=(dataSizeY-kCenter-1)
    for (i = kCenter; i < endIndex; ++i)
    {
        for (k = kSizeY - 1; k >= 0; --k)  // convolve with full kernel
        {
            for (j = 0; j < dataSizeX; ++j)
            {
                sum[j] += *tmpPtr * kernelY[k];
                ++tmpPtr;
            }
        }

        for (n = 0; n < dataSizeX; ++n)  // convert and copy from sum to out
        {
            // covert negative to positive
            *outPtr = (float)((float)fabs(sum[n]) + 0.5f);
            sum[n] = 0;  // reset for next summing
            ++outPtr;    // next output
        }

        // move to next row
        tmpPtr2 += dataSizeX;
        tmpPtr = tmpPtr2;
    }

    // ROW FROM index=(dataSizeY-kCenter) TO index=(dataSizeY-1)
    kOffset = 1;  // ending index of partial kernel varies for each sample
    for (i = endIndex; i < dataSizeY; ++i)
    {
        for (k = kSizeY - 1; k >= kOffset; --k)  // convolve with partial kernel
        {
            for (j = 0; j < dataSizeX; ++j)
            {
                sum[j] += *tmpPtr * kernelY[k];
                ++tmpPtr;
            }
        }

        for (n = 0; n < dataSizeX; ++n)  // convert and copy from sum to out
        {
            // covert negative to positive
            *outPtr = (float)((float)fabs(sum[n]) + 0.5f);
            sum[n] = 0;  // reset for next summing
            ++outPtr;    // next output
        }

        // move to next row
        tmpPtr2 += dataSizeX;
        tmpPtr = tmpPtr2;  // next input
        ++kOffset;         // increase ending index of kernel
    }
    // END OF VERTICAL CONVOLUTION ////////////////////////

    // deallocate temp buffers
    /*delete[] tmp;
    delete[] sum;*/
    free(tmp); free(sum);
    return true;
}

///////////////////////////////////////////////////////////////////////////////
// 1D convolution vertical
///////////////////////////////////////////////////////////////////////////////
bool convV(float* in, float* out, int dataSizeX, int dataSizeY, const float* kernelY, int kSizeY)
{
    // check validity of params
    if (!in || !out || !kernelY)
        return false;
    if (dataSizeX <= 0)
        return false;

    // Save temporary horizontal convolution for the entire image
    int N = dataSizeX * dataSizeY;
    //float* tmpx = new float[N];
    float *tmpx = (float*)calloc(N, sizeof(float));
    if (!tmpx)
        return false;  // memory allocation error
    //for (int i = 0; i < N; ++i)
    //   tmpx[i] = 0;

    // Save temporary vertical convolution for one row
    //float* tmpsum = new float[dataSizeX];
    float *tmpsum = (float*)calloc(dataSizeX, sizeof(float));
    if (!tmpsum)
        return false;  // memory allocation error
    //for (int i = 0; i < dataSizeX; ++i)
      //  tmpsum[i] = 0;

    // find center position of kernel (half of kernel size)
    int kCenter = kSizeY >> 1;           // center index of kernel array
    int endIndex = dataSizeX - kCenter;  // index for full kernel convolution
    int right_half = kSizeY - kCenter - 1; // size of right half right to index 'kCenter' in the kernel

    kCenter = kSizeY >> 1;
    endIndex = dataSizeY - kCenter;
    right_half = kSizeY - kCenter - 1;

    // [0, kCenter - 1]
    int offset = 0;
    for (int j = 0; j < kCenter; ++j)
    {
        for (int k = kCenter + offset, row = 0; k >= 0; k--, row++)
        {
            for (int i = 0; i < dataSizeX; ++i)
            {
                int idx = row * dataSizeX + i;
                tmpsum[i] += tmpx[idx] * kernelY[k];  // tmpsum is 1D row vector
            }
        }
        offset++;

        // Copy tmpSum result to final output image. One 1D row vector 'tmpsum'
        // is enough and this can save storage.
        for (int i = 0; i < dataSizeX; ++i)
        {
            int idx = j * dataSizeX + i;
            out[idx] = (float)tmpsum[i] + 0.5f;
            tmpsum[i] = 0;
        }
    }
    // [kCenter, endIndex - 1]
    for (int j = kCenter; j < endIndex; ++j)
    {
        for (int k = kSizeY - 1, m = 0; k >= 0; k--, m++)
        {
            int row = j - right_half + m;
            for (int i = 0; i < dataSizeX; ++i)
            {
                int idx = row * dataSizeX + i;
                tmpsum[i] += tmpx[idx] * kernelY[k];
            }
        }
        for (int i = 0; i < dataSizeX; ++i)
        {
            int idx = j * dataSizeX + i;
            out[idx] = (float)tmpsum[i] + 0.5f;
            tmpsum[i] = 0;
        }
    }

    // [endIndex, dataSizeY - 1]
    offset = 1;
    for (int j = endIndex; j < dataSizeY; ++j)
    {
        for (int k = kSizeY - 1, m = 0; k >= offset; k--, m++)
        {
            int row = j - right_half + m;
            for (int i = 0; i < dataSizeX; ++i)
            {
                int idx = row * dataSizeX + i;
                tmpsum[i] += tmpx[idx] * kernelY[k];  // tmpsum is 1D row vector
            }
        }
        offset++;
        for (int i = 0; i < dataSizeX; ++i)
        {
            int idx = j * dataSizeX + i;
            out[idx] =(float)tmpsum[i] + 0.5f;
            tmpsum[i] = 0;
        }
    }

    free(tmpx); free(tmpsum);
    return true;
}


bool convH(float* in, float* out, int dataSizeX, int dataSizeY, const float* kernelX, int kSizeX)
{
    int i, j, k, m;
    float *inPtr;   // working pointers
    float *tmpPtr;         // working pointers
    int kCenter, kOffset, endIndex;  // kernel indice

    // check validity of params
    if (!in || !out || !kernelX)
        return false;
    if (dataSizeX <= 0 || kSizeX <= 0)
        return false;

    // covolve horizontal direction ///////////////////////

    // find center position of kernel (half of kernel size)
    kCenter = kSizeX >> 1;           // center index of kernel array
    endIndex = dataSizeX - kCenter;  // index for full kernel convolution

    // init working pointers
    inPtr = in;
    tmpPtr = out;  // store intermediate results from 1D horizontal convolution

    // start horizontal convolution (x-direction)
    for (i = 0; i < dataSizeY; ++i)  // number of rows
    {
        kOffset = 0;  // starting index of partial kernel varies for each sample

        // COLUMN FROM index=0 TO index=kCenter-1
        for (j = 0; j < kCenter; ++j)
        {
            *tmpPtr = 0;  // init to 0 before accumulation

            for (k = kCenter + kOffset, m = 0; k >= 0; --k, ++m)  // convolve with partial of kernel
            {
                *tmpPtr += *(inPtr + m) * kernelX[k];
            }
            ++tmpPtr;   // next output
            ++kOffset;  // increase starting index of kernel
        }

        // COLUMN FROM index=kCenter TO index=(dataSizeX-kCenter-1)
        for (j = kCenter; j < endIndex; ++j)
        {
            *tmpPtr = 0;  // init to 0 before accumulate

            for (k = kSizeX - 1, m = 0; k >= 0; --k, ++m)  // full kernel
            {
                *tmpPtr += *(inPtr + m) * kernelX[k];
            }
            ++inPtr;   // next input
            ++tmpPtr;  // next output
        }

        kOffset = 1;  // ending index of partial kernel varies for each sample

        // COLUMN FROM index=(dataSizeX-kCenter) TO index=(dataSizeX-1)
        for (j = endIndex; j < dataSizeX; ++j)
        {
            *tmpPtr = 0;  // init to 0 before accumulation

            for (k = kSizeX - 1, m = 0; k >= kOffset; --k, ++m)  // convolve with partial of kernel
            {
                *tmpPtr += *(inPtr + m) * kernelX[k];
            }
            ++inPtr;    // next input
            ++tmpPtr;   // next output
            ++kOffset;  // increase ending index of partial kernel
        }

        inPtr += kCenter;  // next row
    }
    // END OF HORIZONTAL CONVOLUTION //////////////////////
    
    return true;
}