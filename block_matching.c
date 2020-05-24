#include "motion.h"
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include "esp_timer.h"

#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h"
#define TAG ""
#else
#include "esp_log.h"
static const char *TAG = "block_matching";
#endif


/** @brief Computes the Mean Absolute Difference (MAD) for the given two blocks
 * 
 * @param currentBlk : The block for which we are finding the MAD of size (n, n)
 * @param refBlk : the block w.r.t. which the MAD is being computed of size (n, n)
 * @param n : the side of the 2 square blcks
 * 
 * @return the MAD for the 2 blks * */
float costFuncMAD(const uint8_t *currentBlk, const uint8_t *refBlk, size_t n) {
    float err = 0.0f;

    for(int i = n - 1; i--; ) {
        for(int j = 0; j < n; j++) 
            err += fabs(currentBlk[j] - refBlk[j]);
        currentBlk += n;
        refBlk += n;
    }
    return err / (float)n / (float)n ;
}

/** @brief Computes the Sum of Absolute Difference (SAD) for the given two blocks
 * 
 * @param currentBlk : The block for which we are finding the SAD of size (n, n)
 * @param refBlk : the block w.r.t. which the SAD is being computed of size (n, n)
 * @param n : the side of the 2 square blcks
 * 
 * @return the SAD for the 2 blks * */
int costFuncSAD(const uint8_t *currentBlk, const uint8_t *refBlk, size_t n) {
    int err = 0;

    for(int i = n - 1; i--; ) {
        for(int j = 0; j < n; j++) 
            err += fabs(currentBlk[j] - refBlk[j]);
        currentBlk += n;
        refBlk += n;
    }
    return err;
}

/** @brief Compute motion compensated image's PSNR
 * 
 * @param imgP : original image of size w * h
 * @param imgComp : compensated image of size w * h
 * @param w : width of image
 * @param h : height of image
 * @param n : the peak value of possible of any pixel in the img
 * 
 * @return motion compensated image's psnr  **/
float imgPSNR(const uint8_t *imgP, const uint8_t *imgComp,\
 size_t w, size_t h, const int n) {
    float err = 0.0f;

    for(int i = h - 1; i--; ) {
        for(int j = 0; j < w; j++) 
            err += pow(imgP[j] - imgComp[j], 2);
        imgP += w;
        imgComp += w;
    }

    const float mse = err / (float)w / (float)h;
    return 10.0f * log10f(n * n / mse);
}

/** @brief Computes motion compensated image using the given motion vectors
 * 
 * @param imgI : reference images of size w * h
 * @param motionVect : the motion vectors of size = w/mbSize * h/mbSize
 * @param w : width of image
 * @param h : height of image
 * @param mbSize : size of the macroblock 
 * 
 * @return motion compensated image of size w * h **/
uint8_t *motionComp(const uint8_t *imgI, const MotionVector16_t *motionVect,\
      size_t w, size_t h, size_t mbSize) {
    // we start off from the top left of the image
    // we will walk in steps of mbSize
    // for every marcoblock that we look at we will read the motion vector
    // and put that macroblock from refernce image in the compensated image
    uint8_t *imgCmp = calloc(w * h, sizeof(uint8_t));
    uint8_t *imageComp = imgCmp;

    if(!imageComp) {
        ESP_LOGE(TAG, "motionComp can't allocate memory");
        return NULL;
    }

    for( int i = 0; i < h - mbSize + 1; i+=mbSize) {
        for( int j = 0; j < w - mbSize + 1; i+=mbSize) {
            const int dx = motionVect->vx;
            const int dy = motionVect->vy;
            const int wdy = dy * w;
            //const refBlkV = i + dy;
            //const refBlkH = j + dx;
            for( int k = mbSize; k--; ) {
                for( int m = 0; m < mbSize; m++) {
                    //imageComp[(k + i) * w + (m + j)] = imgI[(k + refBlkV) * w + (refBlkH + m)];
                    imageComp[m + j] = imgI[wdy + (j + dx + m)]; // faster computation than commented version
                }
                imageComp += w;
                imgI += w;
            }        
            motionVect++;
        }
        imageComp += w;
        imgI += w;
    }
    return imgCmp;
}


/** @brief Computes motion vectors using Adaptive Rood Pattern Search method
 * 
 * @param imgP : image of which we want to find motion vectors  of size w * h
 * @param imgI : reference image of size w * h
 * @param w : width of image
 * @param h : height of image
 * @param mbSize : Size of the macroblock
 * @param p : Search parameter
 * @param zmp_T : Zero-Motion Prejudgement threshold enable if set superior at 0. 
 * improve performance at cost of precision if wrong thresold value.
 * 
 * @param [output] motionVect :  motion vector for each integral macroblock in imgP.  size = w * h/mbSize²
 * @return ARPScomputation :  the avg number of points searched for a macroblock 
 **/
float motionEstARPS(const uint8_t *imgP, const uint8_t *imgI, size_t w, size_t h, size_t mbSize,
 int p, MotionVector16_t *motionVect, int zmp_T) {
     
    motionVect = (MotionVector16_t*) calloc((int)w * h / (float)mbSize / (float)mbSize,
         sizeof(MotionVector16_t));

    if(!motionVect) 
        return -1;
    
    MotionVector16_t *vectors = motionVect;

    // Error window used to computed Minimal Matching Error
    int costs[6] = {INT32_MAX}; 
    float stepSize = 0, maxIndex;

    // The index points for Small Diamond Search pattern
    const int SDSP[6][2] = {{0, -1},
                        {-1, 0},
                        {0, 0},
                        {1, 0},
                        {0, 1},
                        {1, 1}};

    // The index points for Large Diamond Search pattern
    int LDSP[6][2];

    // We will be storing the positions of points where the checking has been already done in an array
    // that is initialised to zero. As one point is checked, we set the corresponding element in the array to one.
    int checkArray[2 * p + 1][2 * p + 1];
    memset(checkArray, 0, sizeof(checkArray[0][0]) * pow(2 * p + 1, 2));

    int computations = 0;

    //mbCount will keep track of how many blocks we have evaluated
    int mbCount = 1;

    // MacroBlocks (MB) used to compute Min Mathing Error (MME) and SAD
    uint8_t *currentBlk = malloc(mbSize * mbSize);
    uint8_t *refBlk = malloc(mbSize * mbSize);
    int i, j, k, m, l, cost, point;
    // we start  off from the top left of image
    // we will walk in step of mbSize
    for(i = 0; i < h - mbSize + 1; i+=mbSize) {
        for(j = 0; j < w - mbSize + 1; j+=mbSize) {
            // the ARPS starts : we are scanning in raster order
            int x = j,
                y = i;

            //                           ##  STEP1:  ##
            //Compute the matching error (SAD) between the current block and the block at the same 
            //location in the ref-erence frame (i.e., the center of the current search window

            // initialise macroblock
            for(k = 0; k < mbSize - 1; k++) {
                for(m = 0; m < mbSize - 1; m++) {
                    currentBlk[k * mbSize + m] = imgP[(k + i) * w + (m + j)];
                    refBlk[k * mbSize + m] = imgI[(k + i) * w + (m + j)];
                }
            }
            costs[2] = costFuncSAD(currentBlk, refBlk, mbSize);

            if(costs[2] < zmp_T) {
                // Zero-Motion Prejud. 
                vectors[mbCount].vx = 0;
                vectors[mbCount].vy = 0;
                vectors[mbCount].mag2 = 0;
                mbCount++;
                continue;
            }

            checkArray[p + 1][p + 1] = 1;
            computations++;

            // if we are in the left most column then we have to make sure that
            // we just do the LDSP with stepSize = 2
            if (!j) {
                stepSize = 2;
                maxIndex = 4;
            } else {
                stepSize = max(abs(vectors[mbCount-1].vx),  abs(vectors[mbCount-1].vy));
                // We check if prediction overlap LDSP in that case we dont recompute
                if( (abs(vectors[mbCount-1].vx) == stepSize && vectors[mbCount-1].vy == 0)
                    || (abs(vectors[mbCount-1].vy) == stepSize && vectors[mbCount-1].vx == 0))
                    maxIndex = 4; //we just have to check at the rood pattern 5 points
                else {
                    maxIndex = 5; //we have to check 6pts
                    LDSP[5][0] = vectors[mbCount-1].vy;
                    LDSP[5][1] = vectors[mbCount-1].vx;
                }
            }

            // The index points for first and only LDSP
            LDSP[0][0] = 0          ; LDSP[0][1] = -stepSize;
            LDSP[1][0] = -stepSize  ; LDSP[1][1] = 0;
            LDSP[2][0] = 0          ; LDSP[2][1] = 0;
            LDSP[3][0] =  stepSize  ; LDSP[3][1] = 0;
            LDSP[4][0] = 0          ; LDSP[4][1] = stepSize;

            // do the LDSP
            //                          ##  STEP 2: ##
            //Align the center of ARP with the center point of the search window and 
            //check its 4 search points (plus the position of the predicted MV if no overlap)
            //to find out the current MME point
            for (k = 0; k < maxIndex; k++) {
                const int refBlkVer = y + LDSP[k][1];
                const int refBlkHor = x + LDSP[k][2];
                if( refBlkVer < 0 || refBlkVer + mbSize > h || 
                        refBlkHor < 0 || refBlkHor + mbSize > w)
                    continue; //outside image boundary
                if (k == 2 || stepSize == 0)
                    continue; //center point already calculated

                // initialise macroblock
                for(l = 0; l < mbSize - 1; l++) {
                    for(m = 0; m < mbSize - 1; m++) {
                        currentBlk[l * mbSize + m] = imgP[(l + i) * w + (m + j)];
                        refBlk[l * mbSize + m] = imgI[(l + refBlkVer) * w + (m + refBlkHor)];
                    }
                }
                costs[k] = costFuncSAD(currentBlk, refBlk, mbSize);
                computations++;
                checkArray[LDSP[k][1] + p + 1][LDSP[k][0] + p + 1] = 1;
            }

            //Find the current MME point
            cost = costs[0], point = 0;
            for(k = 1; k < 6; k++) {
                if (costs[k] < cost) {
                    cost = costs[k];
                    point = k;
                }
            }

            //                         ## STEP 3 ##: 
            //Set  the  center  point  of  the  unit-size  rood  pattern
            //(URP) at the MME point found in the previous step and check its points.

            x += LDSP[point][0];
            y += LDSP[point][1];
            memset(costs, INT32_MAX, 6 * sizeof(int));
            costs[2] = cost;

            //If the new MME point is not incurred at the center of the current URP,
            // repeat this step (step1); otherwise, the MV is found,corresponding to the MME 
            //point identified in this step.Note that in our implementation, a checking 
            //bit-map (one bitfor denoting the status of each macroblock) has been employed
            //to  record  whether  a  search  point  under  checking  has already been 
            //examined before, so that duplicated checking computation can be avoided

            // The doneFlag is set to 1 when the minimum is at the center of the diamond
            // do the SDSP
            int doneFlag = 0;
            while(!doneFlag) {
                for(k = 0; k < 5; k++) {
                    const int refBlkVer = y + SDSP[k][1];
                    const int refBlkHor = x + SDSP[k][0];
                    if( refBlkVer < 0 || refBlkVer + mbSize > h 
                            || refBlkVer < 0 || refBlkHor + mbSize > w)
                            continue;
                    if(k == 2)
                        continue;
                    if(refBlkHor < j-p || refBlkHor > j+p || refBlkVer < i-p || refBlkVer > i+p)
                        continue;
                    if(checkArray[y - i + SDSP[k][1] + p + 1][x - j + SDSP[k][0] + p + 1] == 1)
                        continue;

                    // initialise macroblock
                    for(l = 0; l < mbSize - 1; l++) {
                        for(m = 0; m < mbSize - 1; m++) {
                            currentBlk[l * mbSize + m] = imgP[(l + i) * w + (m + j)];
                            refBlk[l * mbSize + m] = imgI[(l + refBlkVer) * w + (m + refBlkHor)];
                        }
                    }                   
                    costs[k] = costFuncSAD(currentBlk, refBlk, mbSize);
                    checkArray[y - i + SDSP[k][1] + p + 1][x - j + SDSP[k][0] + p + 1] = 1;
                    computations++;
                }

                //Find min of costs and index
                cost = costs[0]; point = 0;
                for(k = 0; k < 5; k++) {
                    if (costs[k] < cost) {
                        cost = costs[k];
                        point = k;
                    }
                }

                if(point == 2) 
                    doneFlag = 1; // Point incurred at the current URP
                else {
                    x += SDSP[point][0]; // else aligne center with SDSP
                    x += SDSP[point][1];
                    memset(costs, INT32_MAX, 6 * sizeof(int));
                    costs[2] = cost;
                }
            }
            //End of step3

            vectors[mbCount].vx = y - i;
            vectors[mbCount].vy = x - j;
            vectors[mbCount].mag2 = powf(vectors[mbCount].vx, 2) + powf(vectors[mbCount].vx, 2);
            mbCount++;
            memset(costs, INT32_MAX, 6 * sizeof(int));
            memset(checkArray, 0, sizeof(checkArray[0][0]) * pow(2 * p + 1, 2));
        }
    }

    free(currentBlk);
    free(refBlk);
    motionVect = &vectors[0];
    return computations / (float)(mbCount - 1);
}