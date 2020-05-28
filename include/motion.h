
#ifndef MOTION_H
#define MOTION_H

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#define max(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })

#define WINDOW 5 //convolution window size for lucas kanade

#define LK_OPTICAL_FLOW_8BIT	0
#define LK_OPTICAL_FLOW 		1
#define BLOCK_MATCHING_ARPS		2

// Motion vector
typedef struct {
	int16_t x, y;
} Vector16_t;

typedef struct {
    int16_t vx, vy;
    uint16_t mag2; // squared magnitude
} MotionVector16_t;

typedef struct {
    int8_t vx, vy;
} MotionVector8_t;

typedef struct MotionEstContext{
	uint8_t *data_cur, *data_ref;       ///< current & prev images
	int method;		  					///< motion estimation method (LK_OPTICAL_FLOW, BLOCK_MATCHING_ARPS, ...)
	char name[20];
	int width, height; 					///< images width and height
	int b_width, b_height, b_count;     ///< blocks width and height
	
	//int xmin, xmax, ymin, ymax; // area mv
	//int pred_x, pred_y; // for certain predictive bma

	// for block matching algo. only
	int mbSize, log2_mbSize; 			///< macro block size
	int search_param; 					///< parameter p in ARPS

	MotionVector16_t *mv_table[3];      ///< motion vectors of current & prev 2 frames
	int max;							///< max motion vector mag²

	bool (*motion_func) (struct MotionEstContext *self);	
} MotionEstContext;

// allocate mv_table motion vector table
bool alloc_mv(MotionEstContext *ctx);
void uninit(MotionEstContext *ctx);
// initialisation TODO generic
bool init_context(MotionEstContext *ctx, int method, int mbSize, int search_param, size_t width, size_t height);


// Motion estimation
bool motion_estimation(MotionEstContext *ctx, uint8_t *img_prev, uint8_t *img_cur);


//						## OPTICAL FLOW

// Lucas Kanade optical flow algorithm
bool LK_optical_flow(const uint8_t *src1, const uint8_t *src2, MotionVector16_t *v, int w, int h, int *max);
bool LK_optical_flow8(const uint8_t *src1, const uint8_t *src2, uint8_t *V, int w, int h);

//#TODO Lucas Kanade DoG (Diff. of Gaussian)
//#TODO Horn-Schunck optical flow algorithm 
//bool HS_optical_flow();


//						## Block matching 

// Adaptative Rood Pattern Search method
bool motionEstARPS(const uint8_t *imgP, const uint8_t *imgI, size_t w, size_t h, size_t mbSize,
 		int p, MotionVector16_t *motionVect, int zmp_T, int *max);

//#TODO : Enhanced Predictive Zonal Search
//bool motionEstEZPS();


//						## TEST METHODS

// Compute motion compensated image
uint8_t *motionComp(const uint8_t *imgI, const MotionVector16_t *motionVect, size_t w, size_t h, 
		size_t mbSize);
// Compute PSNR of image compensated
float imgPSNR(const uint8_t *imgP, const uint8_t *imgComp, size_t w, size_t h, const int n);


#endif