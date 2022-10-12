/** @file */

#ifndef MOTION_H
#define MOTION_H

#ifdef __cplusplus
extern "C"{
#endif 

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#define mmax(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })

#define mmin(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })

/** \brief convolution window size for lucas kanade*/
#define WINDOW 5 

/**
 * @defgroup ALGO_GROUP motion algo
 * @{ \name Algorithm selector
 */
#define LK_OPTICAL_FLOW_8BIT	0
#define LK_OPTICAL_FLOW 	1
#define BLOCK_MATCHING_ARPS	2
#define BLOCK_MATCHING_EPZS     3
/** @} */
	
typedef struct { int16_t x, y; } Vector16_t;

/** @struct MotionVector16_t
* @brief MotionVector 2D with amplitude squared (for speed)
*/
typedef struct {
    int16_t vx, vy;
    uint16_t mag2;
} MotionVector16_t;

/** @struct MotionVector8_t
* @brief MotionVector 2D 8bit
*/
typedef struct {
    int8_t vx, vy;
} MotionVector8_t;

typedef struct MotionEstPredictor {
    int mvs[10][2];
    int nb;
} MotionEstPredictor;


/** @struct MotionEstContext
 *  @brief Exhaustive struct representing all parameter needed for all motion estimation type 
 */
typedef struct MotionEstContext{	
	char name[20];
	uint8_t *data_cur,					///< current image
	        *data_ref;       			///< prev image
	int method;		  					///< motion estimation method (LK_OPTICAL_FLOW, BLOCK_MATCHING_ARPS, ...)
	int width,							///< images width 
	    height, 						///< images height
	    b_width,     					///< blocks width
		b_height, 					    ///< blocks height
		b_count;   	  					///< nb of blocks
	
	//int xmin, xmax, ymin, ymax; // area mv

	int mbSize,							///< macro block size
	    log2_mbSize; 					///< log2 of macro block size
	int search_param; 					///< parameter p in ARPS

	int pred_x,							///< median predictor
		pred_y;                         ///< median predictor
	MotionEstPredictor preds[2];		///< predictor for EPZS

	MotionVector16_t *mv_table[3];      ///< motion vectors of current & prev 2 frames
	int max;							///< max motion vector mag²

	uint64_t (*get_cost) (struct MotionEstContext *self, int x_mb, int y_mb, int x_mv, int y_mv);
	bool (*motion_func) (struct MotionEstContext *self);	
} MotionEstContext;

void uninit(MotionEstContext *ctx);

/** @brief Init & allocate context ctx accordingly to the method used 
	@param ctx : motion estimation context object
 */
bool init_context(MotionEstContext *ctx);

/**
 * @brief motion estimation wrapper
 *  - Will update ctx->name representation of the motion estimation algo used.
 *  - Will call motion estimation algo accordingly (motionEstARPS, motionEstEPS, LK...)
 * \param ctx ctx->method will define which function would be called
 */
bool motion_estimation(MotionEstContext *ctx, uint8_t *img_prev, uint8_t *img_cur);

uint64_t me_comp_sad(MotionEstContext *me_ctx, int x_mb, int y_mb, int x_mv, int y_mv);

	
//						## OPTICAL FLOW
/**
 * @addtogroup ALGO_GROUP 
 * @{ \name Algorithm methods
 */
/** Lucas Kanade optical flow algorithm */
bool LK_optical_flow(const uint8_t *src1, const uint8_t *src2, MotionVector16_t *v, int w, int h, int *max);
bool LK_optical_flow8(const uint8_t *src1, const uint8_t *src2, uint8_t *V, int w, int h);

//#TODO Lucas Kanade DoG (Diff. of Gaussian)
//#TODO Horn-Schunck optical flow algorithm 

//						## Block matching 

/** Adaptative Rood Pattern Search method */
bool motionEstARPS(const uint8_t *imgP, const uint8_t *imgI, size_t w, size_t h, size_t mbSize,
 		int p, MotionVector16_t *motionVect, int zmp_T, int *max);

/** Enhanced Predictive Zonal Search */
bool motionEstEPZS(MotionEstContext *);

/** @} */
	
//						## TEST METHODS

/** Compute motion compensated image */
uint8_t *motionComp(const uint8_t *imgI, const MotionVector16_t *motionVect, size_t w, size_t h, 
		size_t mbSize);
/** Compute PSNR of image compensated */
float imgPSNR(const uint8_t *imgP, const uint8_t *imgComp, size_t w, size_t h, const int n);

#ifdef __cplusplus
}
#endif
#endif
