/** @file block_matching.c
*   Block matching algo
*   @author Thomas Pegot
*/

#include "motion.h"
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include "esp_timer.h"


/** @brief Computes the Sum of Absolute Difference (SAD) for the given two blocks \n
 * 
 * \f[ SAD = \sum_{i=0}^{n-1}\sum_{j=0}^{n-1} |Cur_{ij}-Ref_{ij}| \f]
 *
 * @param currentImg    : img for which we are finding the SAD
 * @param refImg        : img which the SAD is being computed
 * @param offset_curr   : offset applied to img current
 * @param offset_ref    : offset applied to ref img
 * @param mbSize        : the side of the 2 square blcks
 * @param w             : width of images
 * 
 * @return the SAD for the 2 blks * */

int costFuncSAD(const uint8_t *currentImg, const uint8_t *refImg, 
        int offset_curr, int offset_ref, size_t mbSize, size_t w) {
    int err = 0, i, j;

    for(i = 0; i < mbSize; i++) {
        const int iw = i * w ;
        for(j = 0; j < mbSize; j++) 
            err += abs(currentImg[iw + j + offset_curr] - refImg[iw + j + offset_ref]);
    } 
    return err;
}

/*
static float imgPSNR(const uint8_t *imgP, const uint8_t *imgComp,\
 size_t w, size_t h, const int n) {
    float err = 0.0f;

    register int i = w * h;
    while(--i)
        err += pow(*(imgP++) - *(imgComp++), 2);

    const float mse = err / (float)w / (float)h;
    return 10.0f * log10f(n * n / mse);
}
*/

uint8_t *motionComp(const uint8_t *imgI, const MotionVector16_t *motionVect,\
      size_t w, size_t h, size_t mbSize) {
    // we start off from the top left of the image
    // we will walk in steps of mbSize
    // for every marcoblock that we look at we will read the motion vector
    // and put that macroblock from refernce image in the compensated image
    uint8_t *imgCmp = calloc(w * h, sizeof(uint8_t));
    uint8_t *imageComp = imgCmp;

    if(!imageComp) 
        return NULL;

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

// The index points for Small Diamond Search pattern
    const int SDSP[6][2] = {{0, -1},
                        {-1, 0},
                        {0, 0},
                        {1, 0},
                        {0, 1},
                        {1, 1}};

bool motionEstARPS(MotionEstContext *c) {
    // Loading all param
    const uint8_t *imgP = c->data_cur;
    const uint8_t *imgI = c->data_ref;
    const size_t w = (size_t)c->b_width<<c->log2_mbSize;    
    const size_t h = (size_t)c->b_height<<c->log2_mbSize;
    const size_t mbSize = (size_t)c->mbSize;
    const int p = c->search_param;
    MotionVector16_t *vectors = c->mv_table[0];

    // Zero-Motion Prejudgement threshold
    int zmp_T = c->mbSize << (c->log2_mbSize + 1);

    // Error window used to computed Minimal Matching Error
    uint costs[6] = {UINT32_MAX}; 
    uint cost;
    int stepSize = 0, maxIndex = -1;

    // The index points for Large Diamond Search pattern
    int LDSP[6][2];

    // We will be storing the positions of points where the checking has been already done in an array
    // that is initialised to zero. As one point is checked, we set the corresponding element in the array to one.
    int checkArray[2 * p + 1][2 * p + 1];
    memset(checkArray, 0, sizeof(checkArray[0][0]) * pow(2 * p + 1, 2));

    //int computations = 0;
    c->max = 0;
    //mbCount will keep track of how many blocks we have evaluated
    //int mbCount = 0;

    // MacroBlocks (MB) used to compute Min Mathing Error (MME) and SAD
    //uint8_t *currentBlk = malloc(mbSize * mbSize);
    //uint8_t *refBlk = malloc(mbSize * mbSize);

    int i, j, k, point;
    // we start  off from the top left of image
    // we will walk in step of mbSize
    for(i = 0; i < h - mbSize + 1; i+=mbSize) {
        const int iw = i * w;
        for(j = 0; j < w - mbSize + 1; j+=mbSize) {
            // the ARPS starts : we are scanning in raster order
            int x = j,
                y = i;

            //                           ##  STEP1:  ##
            //Compute the matching error (SAD) between the current block and the block at the same 
            //location in the ref-erence frame (i.e., the center of the current search window

            // initialise macroblock  matlab : MB = img(i:i+mbSize-1, j:j+mbSize-1)
            costs[2] = costFuncSAD(imgP, imgI, iw + j, iw + j, mbSize, w);

            if(costs[2] < zmp_T) {
                vectors->vx = 0;  vectors->vy = 0; vectors->mag2 = 0;
                vectors++;
                continue;
            }

            checkArray[p + 1][p + 1] = 1;

            // if we are in the left most column then we have to make sure that
            // we just do the LDSP with stepSize = 2
            if (!j) {
                stepSize = 2;
                maxIndex = 4;
            } else {
                vectors--;
                stepSize = mmax(abs(vectors->vx),  abs(vectors->vy));
                // We check if prediction overlap LDSP in that case we dont recompute
                if( (abs(vectors->vx) == stepSize && vectors->vy == 0)
                    || (abs(vectors->vy) == stepSize && vectors->vx == 0))
                    maxIndex = 4; //we just have to check at the rood pattern 5 points
                else {
                    maxIndex = 5; //we have to check 6pts
                    LDSP[5][0] = vectors->vy;
                    LDSP[5][1] = vectors->vx;
                }
                vectors++;
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
            cost = costs[2], point = 2;
            for (k = 0; k < maxIndex; k++) {
                const int refBlkVer = y + LDSP[k][1];
                const int refBlkHor = x + LDSP[k][0];
                if( refBlkVer < 0 || refBlkVer + mbSize - 1 > h - 1 || 
                        refBlkHor < 0 || refBlkHor + mbSize - 1 > w - 1)
                    continue; //outside image boundary
                if (k == 2 || stepSize == 0)
                    continue; //center point already calculated

                costs[k] = costFuncSAD(imgP, imgI, iw + j, refBlkVer * w + refBlkHor, mbSize, w);
                //computations++;
                checkArray[LDSP[k][1] + p + 1][LDSP[k][0] + p + 1] = 1;                

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
            memset(costs, UINT32_MAX, 6 * sizeof(int));
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
                cost = costs[2]; point = 2;
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
                    if(checkArray[y - i + SDSP[k][1] + p + 1][x - j + SDSP[k][0] + p + 1] == 1) {
                        //Find min of costs and index
                        if (costs[k] < cost) {
                            cost = costs[k];
                            point = k;
                        }
                        continue;
                    }

                    costs[k] = costFuncSAD(imgP, imgI, iw + j, refBlkVer * w + refBlkHor, mbSize, w);
                    checkArray[y - i + SDSP[k][1] + p + 1][x - j + SDSP[k][0] + p + 1] = 1;

                    //Find min of costs and index
                    if (costs[k] < cost) {
                        cost = costs[k];
                        point = k;
                    }
                }

                if(point == 2) 
                    doneFlag = 1; // Point incurred at the current URP
                else {
                    x += SDSP[point][0]; // else align center with SDSP
                    y += SDSP[point][1];
                    memset(costs, UINT32_MAX, 6 * sizeof(int));
                    costs[2] = cost;
                }
            }
            //End of step3
            vectors->vx = y - i;
            vectors->vy = x - j;
            vectors->mag2 = powf(vectors->vx, 2) + powf(vectors->vx, 2);
            c->max = mmax(c->max, vectors->mag2);
            vectors++;
            memset(costs, UINT32_MAX, 6 * sizeof(int));
            memset(checkArray, 0, sizeof(checkArray[0][0]) * pow(2 * p + 1, 2));
        }
    }

    return 1;
}
