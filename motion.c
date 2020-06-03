#include "motion.h"
#include <string.h>
#include <math.h>

bool alloc_mv(MotionEstContext *ctx) {
    int i;
    if(ctx->method == BLOCK_MATCHING_EPZS) {
        for (i = 0; i < 3; i++) {
            ctx->mv_table[i] = (MotionVector16_t*)calloc(ctx->b_count, sizeof(*ctx->mv_table[0]));
            if (!ctx->mv_table[i])
                return 0;
        }
    } else {
        ctx->mv_table[0] = (MotionVector16_t*)calloc(ctx->b_count,  sizeof(*ctx->mv_table[0]));
        if (!ctx->mv_table[0])
            return 0;
    }
    return 1;
}


void freep(void *arg) {
        void *val;

        memcpy(&val, arg, sizeof(val));
        memcpy(arg, &(void *){ NULL }, sizeof(val));
        free(val);
}

void uninit(MotionEstContext *ctx) {
    int i;
    ctx->data_ref = NULL;
    ctx->data_cur = NULL;
    ctx->method = 0;
    
    for (i = 0; i < 3; i++)
       freep(&ctx->mv_table[i]);
}

bool init_context(MotionEstContext *ctx, int method, int mbSize, int search_param,
         size_t width, size_t height) {
    //uninit(ctx);
    ctx->method = method;
    ctx->search_param = search_param;
    ctx->width = width;
    ctx->height = height;
    ctx->get_cost = &me_comp_sad;
    ctx->max = 0;

    //if we define mbSize
    if(mbSize > 0) {
        // redefine as a closest 2^n size
        ctx->log2_mbSize = ceil(log2(mbSize));
        ctx->mbSize = 1 << ctx->log2_mbSize;

        ctx->b_width  = ctx->width  >> ctx->log2_mbSize;
        ctx->b_height = ctx->height >> ctx->log2_mbSize;
        ctx->b_count  = ctx->b_width * ctx->b_height; 

    } else {
        // Macro Block size scale the image size
        ctx->mbSize = (max(ctx->width, ctx->height) > 150) ? 
                    max(ctx->width, ctx->height) / 150 : 1;

        ctx->b_width  = ctx->width  / ctx->mbSize;
        ctx->b_height = ctx->height / ctx->mbSize;
        ctx->b_count  = ctx->b_width * ctx->b_height; 
    }

    return alloc_mv(ctx);
}

static bool LK_optical_flow_wrapper(MotionEstContext *c) {
    return LK_optical_flow(c->data_ref, c->data_cur, c->mv_table[0], c->width,
                 c->height, &c->max);
}

static bool LK_optical_flow8_wrapper(MotionEstContext *c) {
    return LK_optical_flow8(c->data_ref, c->data_cur, c->data_ref, c->width, c->height);
}

static bool motionEstARPS_wrapper(MotionEstContext *c) {
    // Zero-Motion Prejudgement threshold
    int zmp_threshold = c->mbSize << (c->log2_mbSize + 1);

    return motionEstARPS(c->data_cur, c->data_ref, c->b_width << c->log2_mbSize, c->b_height << c->log2_mbSize,
     c->mbSize, c->search_param, c->mv_table[0], zmp_threshold, &c->max);
}

bool motion_estimation(MotionEstContext *ctx, uint8_t *img_prev, uint8_t *img_cur) {
    ctx->data_cur = img_cur;
    ctx->data_ref = img_prev;

    switch (ctx->method)
    {
    case LK_OPTICAL_FLOW        : ctx->motion_func = &LK_optical_flow_wrapper;
        strcpy(ctx->name, "lucas kanade");
        break;
    case LK_OPTICAL_FLOW_8BIT   : ctx->motion_func = &LK_optical_flow8_wrapper;
        strcpy(ctx->name, "lucas kanade 8b");
        break;
    case BLOCK_MATCHING_ARPS    : ctx->motion_func = &motionEstARPS_wrapper;
        strcpy(ctx->name, "ARPS");
        break;    
    case BLOCK_MATCHING_EPZS    : ctx->motion_func = &motionEstEPZS;
        strcpy(ctx->name, "EPZS");
        break;
    default                     : return 0;
    }

    return ctx->motion_func(ctx);
}

uint64_t me_comp_sad(MotionEstContext *me_ctx, int x_mb, int y_mb, int x_mv, int y_mv) {
    uint8_t *data_ref = me_ctx->data_ref;
    uint8_t *data_cur = me_ctx->data_cur;
    int linesize = me_ctx->width;
    uint64_t sad = 0;
    int i, j;

    data_ref += y_mv * linesize;
    data_cur += y_mb * linesize;

    for (j = 0; j < me_ctx->mbSize; j++) {
        const int jlinesize = j * linesize;
        for (i = 0; i < me_ctx->mbSize; i++)
            sad += abs(data_ref[x_mv + i + jlinesize] - data_cur[x_mb + i + jlinesize]);
    }
    return sad;
}

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