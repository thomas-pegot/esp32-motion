/* Taken from https://github.com/FFmpeg/FFmpeg/tree/master/libavfilter 
*/

#include "motion.h"
#include <string.h>
#include <math.h>

#ifndef mid_pred
#define mid_pred mid_pred
static inline const int mid_pred(int a, int b, int c)
{
    if(a>b){
        if(c>b){
            if(c>a) b=a;
            else    b=c;
        }
    }else{
        if(b>c){
            if(c>a) b=c;
            else    b=a;
        }
    }
    return b;
}
#endif

#define COST_MV(x, y)\
do {\
    cost = me_ctx->get_cost(me_ctx, x_mb, y_mb, x, y);\
    if (cost < cost_min) {\
        cost_min = cost;\
        mv[0] = x;\
        mv[1] = y;\
    }\
} while(0)

#define COST_P_MV(x, y)\
if (x >= x_min && x <= x_max && y >= y_min && y <= y_max)\
    COST_MV(x, y);


#define ADD_PRED(preds, px, py)\
    do {\
        preds.mvs[preds.nb][0] = px;\
        preds.mvs[preds.nb][1] = py;\
        preds.nb++;\
    } while(0)

static const int8_t dia1[4][2]  = {{-1, 0}, { 0,-1}, { 1, 0}, { 0, 1}};

/* @brief from https://github.com/FFmpeg/FFmpeg/tree/master/libavfilter
   two subsets of predictors are used
   me->pred_x|y is set to median of current frame's left, top, top-right
   set 1: me->preds[0] has: (0, 0), left, top, top-right, collocated block in prev frame
   set 2: me->preds[1] has: accelerator mv, top, left, right, bottom adj mb of prev frame
*/
uint64_t me_search_epzs(MotionEstContext *me_ctx, int x_mb, int y_mb, int *mv)
{
    int x, y;
    int x_min = max(0, x_mb - me_ctx->search_param);
    int y_min = max(0, y_mb - me_ctx->search_param);
    int x_max = min(x_mb + me_ctx->search_param, (me_ctx->b_width - 1) << me_ctx->log2_mbSize);
    int y_max = min(y_mb + me_ctx->search_param, (me_ctx->b_height - 1) << me_ctx->log2_mbSize);
    uint64_t cost, cost_min;
    int i;

    MotionEstPredictor *preds = me_ctx->preds;

    cost_min = UINT64_MAX;

    COST_P_MV(x_mb + me_ctx->pred_x, y_mb + me_ctx->pred_y);

    if(cost_min < 256)
        return cost_min;

    for (i = 0; i < preds[0].nb; i++)
        COST_P_MV(x_mb + preds[0].mvs[i][0], y_mb + preds[0].mvs[i][1]);

    if(cost_min < 256)
        return cost_min; 

    for (i = 0; i < preds[1].nb; i++)
        COST_P_MV(x_mb + preds[1].mvs[i][0], y_mb + preds[1].mvs[i][1]);

    if(cost_min < 256)
        return cost_min;

    do {
        
        x = mv[0];
        y = mv[1];

        for (i = 0; i < 4; i++)
            COST_P_MV(x + dia1[i][0], y + dia1[i][1]);

    } while (x != mv[0] || y != mv[1]);

    return cost_min;
}

bool motionEstEPZS(MotionEstContext *me_ctx)
{
    int mb_y, mb_x;
    me_ctx->max = 0;

    memcpy(me_ctx->mv_table[2], me_ctx->mv_table[1], sizeof(*me_ctx->mv_table[1]) * me_ctx->b_count);
    memcpy(me_ctx->mv_table[1], me_ctx->mv_table[0], sizeof(*me_ctx->mv_table[0]) * me_ctx->b_count);

    for (mb_y = 0; mb_y < me_ctx->b_height; mb_y++) {
        const int b_line = mb_y * me_ctx->b_width;
        for (mb_x = 0; mb_x < me_ctx->b_width; mb_x++) {
            const int mb_i = mb_x + b_line;
            const int x_mb = mb_x << me_ctx->log2_mbSize;
            const int y_mb = mb_y << me_ctx->log2_mbSize;
            int mv[2] = {x_mb, y_mb};

            MotionEstPredictor *preds = me_ctx->preds;
            preds[0].nb = 0;
            preds[1].nb = 0;

            ADD_PRED(preds[0], 0, 0);
            //left mb in current frame
            if (mb_x > 0)
                ADD_PRED(preds[0], me_ctx->mv_table[0][mb_i - 1].vx, me_ctx->mv_table[0][mb_i - 1].vy);

            //top mb in current frame
            if (mb_y > 0)
                ADD_PRED(preds[0], me_ctx->mv_table[0][mb_i - me_ctx->b_width].vx, me_ctx->mv_table[0][mb_i - me_ctx->b_width].vy);

            //top-right mb in current frame
            if (mb_y > 0 && mb_x + 1 < me_ctx->b_width)
                ADD_PRED(preds[0], me_ctx->mv_table[0][mb_i - me_ctx->b_width + 1].vx, me_ctx->mv_table[0][mb_i - me_ctx->b_width + 1].vy);

            //median predictor
            if (preds[0].nb == 4) {
                me_ctx->pred_x = mid_pred(preds[0].mvs[1][0], preds[0].mvs[2][0], preds[0].mvs[3][0]);
                me_ctx->pred_y = mid_pred(preds[0].mvs[1][1], preds[0].mvs[2][1], preds[0].mvs[3][1]);
            } else if (preds[0].nb == 3) {
                me_ctx->pred_x = mid_pred(0, preds[0].mvs[1][0], preds[0].mvs[2][0]);
                me_ctx->pred_y = mid_pred(0, preds[0].mvs[1][1], preds[0].mvs[2][1]);
            } else if (preds[0].nb == 2) {
                me_ctx->pred_x = preds[0].mvs[1][0];
                me_ctx->pred_y = preds[0].mvs[1][1];
            } else {
                me_ctx->pred_x = 0;
                me_ctx->pred_y = 0;
            }
            //collocated mb in prev frame
            ADD_PRED(preds[0], me_ctx->mv_table[1][mb_i].vx, me_ctx->mv_table[1][mb_i].vy);

            //accelerator motion vector of collocated block in prev frame
            ADD_PRED(preds[1], me_ctx->mv_table[1][mb_i].vx + (me_ctx->mv_table[1][mb_i].vx - me_ctx->mv_table[2][mb_i].vx),
                                me_ctx->mv_table[1][mb_i].vy + (me_ctx->mv_table[1][mb_i].vy - me_ctx->mv_table[2][mb_i].vy));

            //left mb in prev frame
            if (mb_x > 0)
                ADD_PRED(preds[1], me_ctx->mv_table[1][mb_i - 1].vx, me_ctx->mv_table[1][mb_i - 1].vy);

            //top mb in prev frame
            if (mb_y > 0)
                ADD_PRED(preds[1], me_ctx->mv_table[1][mb_i - me_ctx->b_width].vx, me_ctx->mv_table[1][mb_i - me_ctx->b_width].vy);

            //right mb in prev frame
            if (mb_x + 1 < me_ctx->b_width)
                ADD_PRED(preds[1], me_ctx->mv_table[1][mb_i + 1].vx, me_ctx->mv_table[1][mb_i + 1].vy);

            //bottom mb in prev frame
            if (mb_y + 1 < me_ctx->b_height)
                ADD_PRED(preds[1], me_ctx->mv_table[1][mb_i + me_ctx->b_width].vx, me_ctx->mv_table[1][mb_i + me_ctx->b_width].vy);
                
            me_search_epzs(me_ctx, x_mb, y_mb, mv);
            me_ctx->mv_table[0][mb_i].vx = (int16_t) mv[0] - x_mb;
            me_ctx->mv_table[0][mb_i].vy = (int16_t) mv[1] - y_mb;
            me_ctx->mv_table[0][mb_i].mag2 = (uint16_t) pow(mv[0] - x_mb,2) + pow(mv[1] - y_mb,2);
            me_ctx->max = max(me_ctx->max, me_ctx->mv_table[0][mb_i].mag2);
        }
    }
    return 1;
}