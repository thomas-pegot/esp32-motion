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
 * @param currentBlk : The block for which we are finding the MAD
 * @param refBlk : the block w.r.t. which the MAD is being computed
 * @param n : the side of the 2 square blcks
 * 
 * @return the MAD for the 2 blks * */
float costFuncMAD(const uint8_t *currentBlk, const uint8_t *refBlk, size_t n) {
    float err = 0.0f;
    register int i = n * n;
    while(--i) 
        err += fabs(*(currentBlk++) - *(refBlk++));
    return err / (float)n / (float)n ;
}

/** @brief Computes the Sum of Absolute Difference (SAD) for the given two blocks
 * 
 * @param currentBlk : The block for which we are finding the SAD
 * @param refBlk : the block w.r.t. which the SAD is being computed
 * @param n : the side of the 2 square blcks
 * 
 * @return the SAD for the 2 blks * */
int costFuncSAD(const uint8_t *currentBlk, const uint8_t *refBlk, size_t n) {
    int err = 0;
    register int i = n * n;
    while(--i) 
        err += fabs(*(currentBlk++) - *(refBlk++));
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
    /*
    for(int i = h; i--; ) {
        for(int j = 0; j < w; j++) 
            err += pow(imgP[j] - imgComp[j], 2);
        imgP += w;
        imgComp += w;
    }
    */
    register int i = w * h;
    while(--i)
        err += pow(*(imgP++) - *(imgComp++), 2);

    const float mse = err / (float)w / (float)h;
    return 10.0f * log10f(n * n / mse);
}

/** @brief Computes motion compensated image using the given motion vectors
 * 
 * @param imgI : reference images
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
 * @param imgP : image of which we want to find motion vectors 
 * @param imgI : reference image 
 * @param w : width of image
 * @param h : height of image
 * @param mbSize : Size of the macroblock (mbSize, mbSize)
 * @param p : Search parameter
 * @param zmp_T : Zero-Motion Prejudgement threshold enable if set superior at 0. 
 * improve performance at cost of precision if wrong thresold value.
 * 
 * @param [output] motionVect :  motion vector for each integral macroblock in imgP.  size = w * h/mbSize²
 * @return ARPScomputation :  the avg number of points searched for a macroblock 
 **/
float motionEstARPS(const uint8_t *imgP, const uint8_t *imgI, size_t w, size_t h, size_t mbSize,
 int p, MotionVector16_t **MotionVect, int zmp_T) {

    int blck_size = (int)w * h / (mbSize * mbSize);
    MotionVector16_t *output = (MotionVector16_t*) malloc(blck_size * sizeof(MotionVector16_t));

    if(!output) 
        return -1;

    MotionVector16_t *vectors = output;
    //init output
    for(int i = blck_size ; i--; ) {
        vectors->vx = 0;
        vectors->vy = 0;
        vectors->mag2 = 0;
        vectors++;
    }
    vectors = &output[0];

    // Error window used to computed Minimal Matching Error
    uint costs[6] = {UINT32_MAX}; 
    uint cost;
    int stepSize = 0, maxIndex = -1;

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
    //int mbCount = 0;

    // MacroBlocks (MB) used to compute Min Mathing Error (MME) and SAD
    uint8_t *currentBlk = malloc(mbSize * mbSize);
    uint8_t *refBlk = malloc(mbSize * mbSize);

    int i, j, k, m, l, point;
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
            ESP_LOGD(TAG, "vector(%u, %u) = %u", i, j, vectors->mag2);

            // initialise macroblock  matlab : MB = img(i:i+mbSize-1, j:j+mbSize-1)
            for(k = 0; k < mbSize; k++) {
                const int ind_img = k * w + iw + j;
                const int ind_blk = k * mbSize;
                for(m = 0; m < mbSize; m++) {
                    currentBlk[ind_blk + m] = imgP[ind_img + m];
                    refBlk[ind_blk + m] = imgI[ind_img + m];
                }
            }	

            costs[2] = costFuncSAD(currentBlk, refBlk, mbSize);

            if(costs[2] < zmp_T) {
                vectors->vx = 0;  vectors->vy = 0; vectors->mag2 = 0;
                vectors++;
                continue;
            }

            checkArray[p + 1][p + 1] = 1;
            computations++;            
            ESP_LOGD(TAG, "================== STEP1 =================");
            // if we are in the left most column then we have to make sure that
            // we just do the LDSP with stepSize = 2
            if (!j) {
                stepSize = 2;
                maxIndex = 4;
            } else {
                vectors--;
                stepSize = max(abs(vectors->vx),  abs(vectors->vy));
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
            
            ESP_LOGD(TAG, "stepSize = %u", stepSize);
            ESP_LOGD(TAG, "================== STEP2 =================");
            // do the LDSP
            //                          ##  STEP 2: ##
            //Align the center of ARP with the center point of the search window and 
            //check its 4 search points (plus the position of the predicted MV if no overlap)
            //to find out the current MME point
            for (k = 0; k < maxIndex; k++) {
                const int refBlkVer = y + LDSP[k][1];
                const int refBlkHor = x + LDSP[k][0];
                if( refBlkVer < 0 || refBlkVer + mbSize - 1 > h - 1 || 
                        refBlkHor < 0 || refBlkHor + mbSize - 1 > w - 1)
                    continue; //outside image boundary
                if (k == 2 || stepSize == 0)
                    continue; //center point already calculated

                // initialise macroblock
                const int preind_refBlk = refBlkVer * w + refBlkHor;
                for(l = 0; l < mbSize; l++) {
                    const int ind_blk = l * mbSize;
                    const int ind_imgP = l * w + iw + j;
                    const int ind_imgI = l * w + preind_refBlk;
                    for(m = 0; m < mbSize; m++) {
                        currentBlk[ind_blk + m] = imgP[ind_imgP + m];
                        refBlk[ind_blk + m] = imgI[ind_imgI + m];
                    }
                }
                costs[k] = costFuncSAD(currentBlk, refBlk, mbSize);
                computations++;
                checkArray[LDSP[k][1] + p + 1][LDSP[k][0] + p + 1] = 1;                
                ESP_LOGV(TAG, "blckV = %i, blckH = %i, cost = %u", refBlkVer, refBlkHor, costs[k]);
            }            

            //Find the current MME point
            cost = costs[0], point = 0;
            for(k = 0; k < 5; k++) {
                if (costs[k] < cost) {
                    cost = costs[k];
                    point = k;
                }
            }

            ESP_LOGD(TAG, "MIN : cost = %u, point = %u", cost, point);
            ESP_LOGD(TAG, "================== STEP3 =================");
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
                    const int preind_refBlk = refBlkVer * w + refBlkHor;
                    for(l = 0; l < mbSize; l++) {
                        const int ind_blk = l * mbSize;
                        const int ind_imgP = l * w + iw + j;
                        const int ind_imgI = l * w + preind_refBlk;
                        for(m = 0; m < mbSize; m++) {
                            currentBlk[ind_blk + m] = imgP[ind_imgP + m];
                            refBlk[ind_blk + m] = imgI[ind_imgI + m];
                        }
                    }

                    costs[k] = costFuncSAD(currentBlk, refBlk, mbSize);
                    checkArray[y - i + SDSP[k][1] + p + 1][x - j + SDSP[k][0] + p + 1] = 1;
                    computations++;
                    ESP_LOGV(TAG, "blckV = %i, blckH = %i, cost = %u", refBlkVer, refBlkHor, costs[k]);
                }

                //Find min of costs and index
                cost = costs[0]; point = 0;
                for(k = 0; k < 5; k++) {
                    if (costs[k] < cost) {
                        cost = costs[k];
                        point = k;
                    }
                }

                ESP_LOGD(TAG, "MIN : cost = %u, point = %u", cost, point);
                if(point == 2) 
                    doneFlag = 1; // Point incurred at the current URP
                else {
                    x += SDSP[point][0]; // else align center with SDSP
                    x += SDSP[point][1];
                    memset(costs, UINT32_MAX, 6 * sizeof(int));
                    costs[2] = cost;
                }
            }
            //End of step3
            vectors->vx = y - i;
            vectors->vy = x - j;
            vectors->mag2 = powf(vectors->vx, 2) + powf(vectors->vx, 2);
            vectors++;
            memset(costs, UINT32_MAX, 6 * sizeof(int));
            memset(checkArray, 0, sizeof(checkArray[0][0]) * pow(2 * p + 1, 2));
        }
    }

    free(currentBlk);
    free(refBlk);

    *MotionVect = output;

    return computations / (float)(blck_size - 1);
}