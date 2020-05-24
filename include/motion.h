
#ifndef MOTION_H
#define MOTION_H

#define WINDOW 5 //convolution window size for lucas kanade
#define max(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>


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


//#TODO Post processing motion filtering
/*
// Limit min magnitude and number min magnitude to filter
extern int motion_mag2_limit;
extern int motion_mag2_limit_count;

typedef struct {
    int vx,
        vy;

    int mag2,           // Magnitude² of the composite vector
        mag2_count;     // number of motion vect added to this composite

    int box_w,          // A box around the cvec for testing
        box_h,          // the vector concentration composing the cvec 
        in_box_count;   // count of motion vect inside the box
        //in_box_rejects;

} CompositeVector;
*/

//## Lucas Kanade optical flow alggorithm
bool LK_optical_flow(const uint8_t *src1, const uint8_t *src2, MotionVector16_t *v, int w, int h);
bool LK_optical_flow8(const uint8_t *src1, const uint8_t *src2, uint8_t *V, int w, int h);

//## Block matching motion detection

//# Adaptative Rood Pattern Search method
float motionEstARPS(const uint8_t *imgP, const uint8_t *imgI, size_t w, size_t h, size_t mbSize,
 int p, MotionVector16_t *motionVect, int zmp_T);

// Compute motion compensated image
uint8_t *motionComp(const uint8_t *imgI, const MotionVector16_t *motionVect, size_t w, size_t h, size_t mbSize);
// Compute PSNR of imaage compensated
float imgPSNR(const uint8_t *imgP, const uint8_t *imgComp, size_t w, size_t h, const int n);



#endif

//#TODO in motion.c
/* If we are left with enough counts for a composite vector, filter out
|  motion vectors not pointing in the composite directon.
|  Vectors of sufficient mag2 but not pointing in the right direction
|  will be rejects and their count can be large for noisy frames.
|  Dot product to allow a spread,
|    (avoiding sqrt() to get magnitude and scale by 100 for integer math):
|
|  cos(a) = (v1 dot v2) / mag(v1) * mag(v2))	# cos(25) = 0.906
|  100 * cos(25)^2 = 82 = 100 * (v1 dot v2)^2 / mag(v1)^2 * mag(v2)^2)
*/
// In construction ... 
/*
bool filterSpread(MotionVector16_t *V, uint8_t *output, int w, int h) {

	MotionVector16_t *mv;
	//int trigger[w * h];
	int i, j;
	// First pass, filter out any vector < mag2_limit
	

	for (i = half_window; i < h - half_window; ++i) {
		for (j = half_window; j < w - half_window; ++j) {
			const int mb_index = w * i + j;
			mv = &V[mb_index];
			const int mag2 = mv->vx * mv->vx + mv->vy * mv->vy;
			if (mag2 >= motion_mag2_limit)
				*(output + mb_index) = mag2;
		}
	}
	return true;

}*/