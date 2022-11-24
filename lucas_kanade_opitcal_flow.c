/** @file lucas_kanade_optical_flow.c
*   @brief Lucas Kanade implementation of optical flow in 16 and 8 bit
*
*   
*   @author Thomas Pegot 
*/

#include "motion.h"
#include "convolution.h"
#include <stdbool.h>
#include "assert.h"
#include <math.h>
#include "esp_heap_caps.h" 
#include <string.h> //memcpy, memset

#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h"
#define TAG ""
#else
#include "esp_log.h"
static const char *TAG = "LK_OPTICAL_FLOW";
#endif

/** don't include Gaussian smooth for faster result (can allow if a denoise is already done in JPEG decomp)*/
#define NOSMOOTH 1

static const float NoiseThreshold = 0.01; /* Lucas Kanade noise threshold */
static const int half_window = WINDOW << 1;
static const int window_squared = WINDOW * WINDOW;
static const int log2_window = (int const)ceil(log2(WINDOW));

/** define 5x5 Gaussian kernel flattened */
static const float kernel[WINDOW * WINDOW] = {1 / 256.0f, 4 / 256.0f, 6 / 256.0f, 4 / 256.0f, 1 / 256.0f, 4 / 256.0f, 16 / 256.0f,
	24 / 256.0f, 16 / 256.0f, 4 / 256.0f, 6 / 256.0f, 24 / 256.0f, 36 / 256.0f, 24 / 256.0f, 6 / 256.0f, 4 / 256.0f,
	16 / 256.0f, 24 / 256.0f, 16 / 256.0f, 4 / 256.0f, 1 / 256.0f, 4 / 256.0f, 6 / 256.0f, 4 / 256.0f, 1 / 256.0f};

/** Separable Gaussian kernel */
static const float Kernel_isotropic[WINDOW] = {1.0 / 16.0, 4.0 / 16.0, 6.0 / 16.0, 4.0 / 16.0, 1.0 / 16.0 };

/* Differentiate Lucas Kanade kernel https://www.cs.toronto.edu/~fleet/research/Papers/ijcv-94.pdf */
static const float Kernel_Dxy[WINDOW] = {-1.0 / 12.0, 8.0 / 12, 0, -8.0 / 12.0, 1.0 / 12.0};

static void *_malloc(size_t size) {
    void *res = malloc(size);
    if(res)
        return res;
    return heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
}

bool LK_optical_flow(MotionEstContext *ctx) {
	const int w = ctx->width;
	const int h = ctx->height;
	const int N = w * h;
	float *image1 = (float*)_malloc(N * sizeof(float));   // temp image
#if !NOSMOOTH
	float *image2 = (float*)_malloc(N * sizeof(float)); // temp image
	if(!image2) {
		ESP_LOGE(TAG, "allocation failed!");
		return false;
	}
#endif
	float *fx = (float*)_malloc(N * sizeof(float)),
		*ft = (float*)_malloc(N * sizeof(float)),
		*fy = (float*)_malloc(N * sizeof(float));
	int i, j, m;
	ctx->max = 0;

	if(!fx || !fy || !ft || !image1) {
		ESP_LOGE(TAG, "allocation failed!");
		return false;
	}

	/* init input */
	for(i = N; i--; ) {
		const float tmp_fX = ctx->data_ref[i];
		fx[i] = tmp_fX;
		ft[i] = ctx->data_cur[i] - tmp_fX;  /* Gradient computation: I_{t+1} - I_{t} */
		fy[i] = tmp_fX;   					/* fy initialisation as smoothed input = fx */

		ctx->mv_table[0][i].vx = 0;
		ctx->mv_table[0][i].vy = 0;
		ctx->mv_table[0][i].mag2 = 0;
	}

#if NOSMOOTH
	/* Derivate Dx : 1D convolution horizontal */
	if(!convH(fx, image1, w, h, Kernel_Dxy, 5)) {
		ESP_LOGE(TAG, "convH failed!");
		return false;
	}
	memcpy(fx, image1, N * sizeof(float));

	/* Derivate Dy : 1D convolution vertical */
	if(!convV(fy, image1, w, h, Kernel_Dxy, 5)) {
		ESP_LOGE(TAG, "convV failed!");
		return false;
	}
	memcpy(fy, image1, N * sizeof(float));
#else
	/* Derivate Dx : 1D convolution horizontal */
	if(!convH(fx, image1, w, h, Kernel_Dxy, 5)) {
		ESP_LOGE(TAG, "convH failed!");
		return false;
	}
	
	/* Derivate Dy : 1D convolution vertical */
	if(!convV(fy, image2, w, h, Kernel_Dxy, 5)) {
		ESP_LOGE(TAG, "convV failed!");
		return false;
	}

	/* Isotropic smooth */
	if(!convolve2DSeparable(image1, fx, w, h, Kernel_isotropic, 5, Kernel_isotropic, 5)) {
		ESP_LOGE(TAG, "convolve2DSeparable failed!") ;
		return false;
	}

	if(!convolve2DSeparable(image2, fy, w, h, Kernel_isotropic, 5, Kernel_isotropic, 5)) {
		ESP_LOGE(TAG, "convolve2DSeparable failed!") ;
		return false;
	}

	if(!convolve2DSeparable(ft, image2, w, h, Kernel_isotropic, 5, Kernel_isotropic, 5)) {
		ESP_LOGE(TAG, "convolve2DSeparable failed!") ;
		return false;
	}
	
	memcpy(ft, image2, N * sizeof(float));
#endif

	// Lucas Kanade optical flow algorithm
	for(i = half_window; i < h - half_window; ++i) {
		for(j = half_window; j < w - half_window; ++j) {
			float Atb0 = 0, Atb1 = 0; 
			float a = 0, b = 0, c = 0;
			for(m = 0; m < window_squared; ++m) {
				// Sum over the window W²
				const float W = kernel[m];
				const int i_window = (m >> log2_window) - half_window;
				const int j_window = (m % WINDOW) - half_window;
                const unsigned index = (j + j_window) + (i + i_window) * w;
				const float Ix = (float) fx[index] * W;
				const float Iy = (float) fy[index] * W;
				const float It = (float) ft[index] * W;
				a += Ix * Ix;
				c += Iy * Iy; 
				b += Ix * Iy;
				Atb0 += - Ix * It;
				Atb1 += - Iy * It;
			}
			
			//const float eigenval1 = ((a + c) + sqrtf(4 * b * b + powf(a - c, 2))) /2;
			//const float eigenval2 = ((a + c) - hypotf(2 * b, a - c)) * 0.5;

			if(((a + c) - hypotf(2 * b, a - c)) * 0.5 >= NoiseThreshold) {
				//Case 1: λ1≥λ2≥τ equivalent to λ2≥τ
				//A is nonsingular, the system of equations are solved using Cramer's rule.
				const float det = a * c - b * b;
				const float b_by_det = b / det;
				const float iAtA[2][2] = {{  c / det, - b_by_det},
										  {- b_by_det,   a / det}};
				//optical flow : [Vx Vy] = inv[AtA] . Atb
				const float vx = iAtA[0][0] * Atb0 + iAtA[0][1] * Atb1;
				const float vy = iAtA[1][0] * Atb0 + iAtA[1][1] * Atb1;	
				MotionVector16_t *mv = &ctx->mv_table[0][i * w + j];
				mv->vx = (int16_t)vx;
				mv->vy = (int16_t)vy;
				mv->mag2 = (uint16_t)(vx * vx + vy * vy);	
				if(ctx->max < mv->mag2)
					ctx->max = mv->mag2;		
			} 
		}
    }

	free(fx); free(image1); free(ft); free(fy); 
#if !NOSMOOTH
	free(image2);
#endif
	return true;
}

bool LK_optical_flow8(const uint8_t *src1, const uint8_t *src2, uint8_t *out, int w, int h) {
	const int N = w * h;
	uint16_t *tmpMagArray = (uint16_t*)calloc(N, sizeof(uint16_t));
	uint16_t *pMag = tmpMagArray;
	uint16_t maxMag = 0;
	float *image1 = (float*)_malloc(N * sizeof(float)),
		*image2 = (float*)_malloc(N * sizeof(float)),
		*fx = (float*)_malloc(N * sizeof(float)),
		*ft = (float*)_malloc(N * sizeof(float)),
		*fy = (float*)_malloc(N * sizeof(float));
	int i, j, m;

	if(!fx || !fy || !ft || !image1 || !image2 || !N || !src1 || !src2) 
		return false;

	// init input
	for(i = N; i--; ) {
		const float tmp_fX = src1[i];
		fx[i] = tmp_fX;
		ft[i] = src2[i] - tmp_fX;  /* Gradient computation: I_{t+1} - I_{t} */
		fy[i] = tmp_fX;   /* fy initialisation as smoothed input = fx */
	}	

	// Derivate Dx : 1D convolution horizontal
	if(!convH(fx, image1, w, h, Kernel_Dxy, 5)) {
		return false;
	}
	
	// Derivate Dy : 1D convolution vertical
	if(!convV(fy, image2, w, h, Kernel_Dxy, 5)) {
		return false;
	}

	// ##Isotropic smooth
	if(!convolve2DSeparable(image1, fx, w, h, Kernel_isotropic, 5, Kernel_isotropic, 5)) {
		return false;
	}

	if(!convolve2DSeparable(image2, fy, w, h, Kernel_isotropic, 5, Kernel_isotropic, 5)) {
		return false;
	}
	
	if(!convolve2DSeparable(ft, image2, w, h, Kernel_isotropic, 5, Kernel_isotropic, 5)) {
		return false;
	}
	
	memcpy(ft, image2, N * sizeof(float));

	memset(out, 0, N);
	// Lucas Kanade optical flow algorithm
	for(i = half_window; i < h - half_window; ++i) {
		for(j = half_window; j < w - half_window; ++j) {
			float Atb0 = 0, Atb1 = 0; 
			float a = 0, b = 0, c = 0;
			for(m = 0; m < window_squared; ++m) {
				// Sum over the window W²
				const float W = kernel[m];
				const int i_window = m / WINDOW - half_window;
				const int j_window = m % WINDOW - half_window;
                const unsigned index = (j + j_window) + (i + i_window) * w;
				const float Ix = (float) fx[index] * W;
				const float Iy = (float) fy[index] * W;
				const float It = (float) ft[index] * W;
				a += Ix * Ix;
				c += Iy * Iy; 
				b += Ix * Iy;
				Atb0 += - Ix * It;
				Atb1 += - Iy * It;
			}
			
			//const float eigenval1 = ((a + c) + sqrtf(4 * b * b + powf(a - c, 2))) /2;
			const float eigenval2 = ((a + c) - hypotf(2 * b, a - c)) * 0.5;

			if(eigenval2 >= NoiseThreshold) {
				//Case 1: λ1≥λ2≥τ equivalent to λ2≥τ
				//A is nonsingular, the system of equations are solved using Cramer's rule.
				const float det = a * c - b * b;
				const float b_by_det = b / det;
				const float iAtA[2][2] = {{  c / det, - b_by_det},
										  {- b_by_det,   a / det}};
				//optical flow : [Vx Vy] = inv[AtA] . Atb
				const float vx = iAtA[0][0] * Atb0 + iAtA[0][1] * Atb1;
				const float vy = iAtA[1][0] * Atb0 + iAtA[1][1] * Atb1;	
				pMag = &tmpMagArray[i * w + j] ;
				*pMag = (uint16_t)(vx * vx + vy * vy);
				if(*pMag > maxMag)
					maxMag = *pMag;
			}
		}
    }

	for(pMag = &tmpMagArray[0], i = 0; i < N; i++, pMag++)
		out[i] = (uint8_t)(*pMag * 255.0 / (float)maxMag);

	free(fx); free(image1);
	free(fy); free(image2);
	free(ft); free(tmpMagArray);
	return true;
}
