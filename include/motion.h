/** @file motion.c
*   motion header
*   @author Thomas Pegot
*/

#ifndef MOTION_H
#define MOTION_H

#ifdef __cplusplus
extern "C"{
#endif 

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

/** @brief return max as the same type of input */
#define mmax(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })

/** @brief return min as the same type of input */
#define mmin(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })

/** \brief convolution window size for lucas kanade*/
#define WINDOW 5 

/**
 * @name Algorithm selector
 * @{ 
 */
#define LK_OPTICAL_FLOW_8BIT	0
#define LK_OPTICAL_FLOW 	1
#define BLOCK_MATCHING_ARPS	2
#define BLOCK_MATCHING_EPZS     3
/** @} */
	
typedef struct { int16_t x, y; } Vector16_t;

/**
 * @struct MotionVector16_t
 * @brief MotionVector 2D with amplitude squared (for speed)
 */
typedef struct {
    int16_t  vx;	 /*!< guess..*/
	int16_t  vy;	 /*!< guess..*/
    uint16_t mag2;   /*!< Squared magnitude $mag2 = vx^2 + vy^2$*/
} MotionVector16_t;

/** 
 * @struct MotionVector8_t
 * @brief MotionVector 2D 8bit
 */
typedef struct {
    int8_t  vx;		 /*!< guess..*/
	int8_t  vy;		 /*!< guess..*/
} MotionVector8_t;

/** 
 * @struct MotionEstPredictor
 * @brief Used for EPZS algorithm
 */
typedef struct MotionEstPredictor {
    int mvs[10][2];  /*!< table of mv predictor    */
    int nb;			 /*!< number of predictor added*/
} MotionEstPredictor;


/** 
 * @struct MotionEstContext
 *  @brief Exhaustive struct representing all parameter needed for all motion estimation type 
 */
typedef struct MotionEstContext{	
	char name[20];
	uint8_t *data_cur,					///< current image
	        *data_ref;       			///< prev image
	int method;		  					///< motion estimation method (LK_OPTICAL_FLOW, BLOCK_MATCHING_ARPS, ...)
	int max;							///< max motion vector mag²
	int width,							///< images width 
	    height, 						///< images height
	//	linewidth,						///< images width * depth (= width * 3 if rgb)	
    /**
     * @name Block Matching (EPZS, ARPS) element related 
     * @{
     */
	    b_width,     					///< blocks width
		b_height, 					    ///< blocks height
		b_count;   	  					///< nb of blocks
	
	int mbSize,							///< macro block size
	    log2_mbSize; 					///< log2 of macro block size
	int search_param; 					///< parameter p in ARPS

	int pred_x,							///< median predictor
		pred_y;                         ///< median predictor
	MotionEstPredictor preds[2];		///< predictor for EPZS

	MotionVector16_t *mv_table[3];      ///< motion vectors of current & prev 2 frames
	/** @} */

	/** pointer to motion estimation function */
	uint64_t (*get_cost) (struct MotionEstContext *self, int x_mb, int y_mb, int x_mv, int y_mv);
	bool (*motion_func) (struct MotionEstContext *self);	
} MotionEstContext;

void uninit(MotionEstContext *ctx);

/** 
 * @brief Init & allocate context ctx accordingly to the method used 
 * @param ctx : motion estimation context object
 * @return big if true
 */
bool init_context(MotionEstContext *ctx);

/**
 * @brief motion estimation wrapper
 *  - Will update ctx->name representation of the motion estimation algo used.
 *  - Will call motion estimation algo accordingly (motionEstARPS, motionEstEPS, LK...)
 *  - current Motion Vector will be saved in mv_table[0].
 * 
 * @note Some algo will save previous motion into mv_table[1] and mv_table[2]
 * 
 * 	Example usage :
 * @code{c}
 * MotionEstContext * motion(uint8_t *img_prev, uint8_t *img_cur, size_t w, size_t h)
 * {
 *     // Generate a motion context configuration 
 *     MotionEstContext me_ctx = {.method = LK_OPTICAL_FLOW,    // algorithm used
 *                                .width  = w,  .height = h // size of your image
 *                               };
 *     // Init the context 
 *     init_context(&me_ctx);
 * 
 *     // Start the motion estimation
 *     if(!motion_estimation(&me_ctx, (uint8_t *)img_prev, (uint8_t *)img_cur)) {
 *         // If motion fail return 0
 *         printf("error");
 * 
 *     return &me_ctx;
 * }
 * @endcode
 * 
 * @param ctx Motion vectors will be saved in (MotionVector16_t) ctx->mv_table[0]
 * 
 * @return          Big if True
 */
bool motion_estimation(MotionEstContext *ctx, uint8_t *img_prev, uint8_t *img_cur);

/**
 * @brief motion estimation compasation sum absolute difference
 * 
 * @param me_ctx 
 * @param x_mb 
 * @param y_mb 
 * @param x_mv 
 * @param y_mv 
 * @return uint64_t 
 */
uint64_t me_comp_sad(MotionEstContext *me_ctx, int x_mb, int y_mb, int x_mv, int y_mv);

/**
 * @name Algorithm methods
 * @addtogroup ALGO_GROUP 
 * @{ 
 */

/**
 * @brief Implement LK optical flow source from wiki and matlab : https://en.wikipedia.org/wiki/Lucas%E2%80%93Kanade_method   \n
 *
 * This method is based on *Taylor series* resolution: \n
 *
 * \f[ I(x+\Delta x,y+\Delta y,t+\Delta t) = I(x,y,t) + \frac{\partial I}{\partial x}\,\Delta x+\frac{\partial I}{\partial y}\,\Delta y+\frac{\partial I}{\partial t} \, \Delta t+{} \f]
 * Which can be rewritten:  \n
 *
 * \f[ \frac{\partial I}{\partial x}V_x+\frac{\partial I}{\partial y}V_y+\frac{\partial I}{\partial t} = 0 \f]
 * Lucas Kanade reorder this equation as matrix such as:
    \f[ A = \begin{bmatrix}
	I_x(q_1) & I_y(q_1) \\[10pt]
	I_x(q_2) & I_y(q_2) \\[10pt]
	\vdots & \vdots \\[10pt]
	I_x(q_n) & I_y(q_n) 
	\end{bmatrix}
	\quad\quad\quad
	v = 
	\begin{bmatrix}
	V_x\\[10pt]
	V_y
	\end{bmatrix}
	\quad\quad\quad
	b = 
	\begin{bmatrix}
	-I_t(q_1) \\[10pt]
	-I_t(q_2) \\[10pt]
	\vdots \\[10pt]
	-I_t(q_n)
	\end{bmatrix} \f]
	
 * Then the solution can be reduced as : \f$ A^T A v=A^T b \f$ or \f$ \mathrm{v}=(A^T A)^{-1}A^T b \f$
 * @param[in] src1  pointer to grayscale buffer image instant t. 
 * @param[in] src2  pointer to grayscale buffer image instant t+1.
 * @param[out] v    vector (vx, vy) and squared magnitude
 * @param[in] w     width
 * @param[in] h     height
 * @param[out] max  max squared magnitude of all motion vector
 * 
 * @return          Big if True
 */
bool LK_optical_flow(const uint8_t *src1, const uint8_t *src2, MotionVector16_t *v, int w, int h, int *max);

/**
 * @brief Lucas Kanade optical 8 bit version 
 *
 *  Same as LK_optical_flow except the output is a uint8 table instead of vector16. 
 *  Then only the squared magnitude will be saved (no vx, vy) and normalize to [0..255]
 * 
 * @param[in] src1  pointer to grayscale buffer image instant t. 
 * @param[in] src2  pointer to grayscale buffer image instant t+1.
 * @param[in] w     width
 * @param[in] h     height
 * @param [out] v   image 8bit depth output of squared magnitude
 * 
 * @return          Big if True
 */
bool LK_optical_flow8(const uint8_t *src1, const uint8_t *src2, uint8_t *v, int w, int h);

/**
 * @brief Perform Adaptive Rood Pattern Search algorithm
 * 
 * @param imgP    image of which we want to find motion vectors 
 * @param imgI    reference image 
 * @param w       width of image
 * @param h       height of image
 * @param mbSize  Size of the macroblock (mbSize, mbSize)
 * @param p       Search parameter
 * @param zmp_T   Zero-Motion Prejudgement threshold enable if set superior at 0. 
 * improve performance at cost of precision if wrong thresold value.
 * 
 * @param [out] motionVect      motion vector for each integral macroblock in imgP.  size = w * h/mbSize²
 * @param [out] max             max squared magnitude of all motion vectors
 * @param [out] ARPScomputation @todo the avg number of points searched for a macroblock 
 * 
 * @return Big if true
 */
bool motionEstARPS(const uint8_t *imgP, const uint8_t *imgI, size_t w, size_t h, size_t mbSize,
 		int p, MotionVector16_t *motionVect, int zmp_T, int *max);

/**
 * @brief Enhance Predictive Zonal Search block matching algo.
 * 
 * Implemented from article DOI: 10.15406/oajs.2017.01.00002
 * 
 * @note : `me_search_epzs` function is taken from ffmpeg libavfilter and rearrange accordingly
 * @todo : I bypassed some error by a little trick `if cost_min < 256` (if I remember). this might cause
 * 		   artifacts. I have to investigate on this.. 
 * 
 * @param me_ctx     Motion estimation context with me_ctx->method = 'BLOCK_MATCHING_EPZS'
 * 
 * @return           big if true 
 * */
bool motionEstEPZS(MotionEstContext *);

/** @} */

/** 
 * @brief Compute motion compensated image's PSNR  \n
 *  \f[ \text{PSNR} = 10 \log_{10}\frac {(\text{peak to peak value of original data})^2}{\text{MSE}}  \f]
 * @param imgP      : original image of size w * h
 * @param imgComp   : compensated image of size w * h
 * @param w         : width of image
 * @param h         : height of image
 * @param n         : the peak value of possible of any pixel in the img
 * 
 * @return motion compensated image's psnr  
 */
uint8_t *motionComp(const uint8_t *imgI, const MotionVector16_t *motionVect, size_t w, size_t h, 
		size_t mbSize);
/** Compute PSNR of image compensated */
float imgPSNR(const uint8_t *imgP, const uint8_t *imgComp, size_t w, size_t h, const int n);

#ifdef __cplusplus
}
#endif
#endif
