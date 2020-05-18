
#ifndef MOTION_H
#define MOTION_H

//convolution window size
#define WINDOW 5

#include <stdint.h>
#include <stdbool.h>
/*
extern const float NoiseThreshold;// Lucas Kanade noise threshold
//extern const int window; // size of window
// define 5x5 Gaussian kernel flattened
extern const float kernel[WINDOW * WINDOW];
// Separable Gaussian kernel
extern const float Kernel_Hgaussian[WINDOW];
extern const float Kernel_Vgaussian[WINDOW];
extern const float Kernel_isotropic[WINDOW]; // same as gauss
// Differentiate Lucas Kanade kernel https://www.cs.toronto.edu/~fleet/research/Papers/ijcv-94.pdf
extern const float Kernel_Dxy[WINDOW];
// Limit min magnitude and number min magnitude to filter
extern int motion_mag2_limit;
extern int motion_mag2_limit_count;
*/
// Motion vector
typedef struct {
    int16_t vx,
            vy;
    uint16_t mag2; // squared magnitude
} MotionVector16_t;

typedef struct {
    int8_t vx,
            vy;
} MotionVector8_t;


// Composite vector TODO state
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


bool LK_optical_flow(const uint8_t *src1, const uint8_t *src2, MotionVector16_t *v, int w, int h);
bool LK_optical_flow8(const uint8_t *src1, const uint8_t *src2, uint8_t *V, int w, int h);

#endif