/** @file deflicker.c
 * Deflickering algorithm
 * @author Thomas Pegot
 */

#include "convolution.h"
#include "deflicker.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <assert.h>

static queue_t queue = { .brightness = {0.0f}, .available = 0 };
static queue_t *q = &queue;

/** @brief Fast clipping */
static uint8_t clip_uint8(int a)
{
     if (a&(~0xFF)) return (-a)>>31;
     else           return a;
}

/** @brief calculate brightness as the average pixel val
*   @param img pointer to img
*   @param size size of the image
*   @return average (float)
*/
float calc_brightness(uint8_t *img, int size) {
    int i;
    float sum = 0;

        for (i = 0; i < size; i++) 
            sum += img[i];

    return sum / (float)size;
}

float get_factor() {
    int i;
    float sum = 0.0f;
    for( i = 0; i < MAXSIZE; i++) 
        sum += q->brightness[i];

    sum /=  (float)MAXSIZE;
    return sum / q->brightness[MAXSIZE - 1];
}

bool deflicker(uint8_t *img, int w, int h) {

    const int size = w * h;
    float f = 0;
    int i;

    float currBrightness  = calc_brightness(img, size);

    // Calculate brightness of current image and stack queue 
    if(q->available < MAXSIZE) {
        // Fill the queue first and don't deflicker.
        q->brightness[q->available] = currBrightness;
        q->available++;
        if(q->available < MAXSIZE) // While queue not filled don't deflicker
            return false;
    } else {
        // Push into filled queue FIFO 
        memmove(&q->brightness[0], &q->brightness[1], sizeof(*q->brightness) * (MAXSIZE - 1));
        q->brightness[MAXSIZE - 1] = currBrightness;
    }

    f = get_factor();

    for (i = 0; i < size; i++)
        img[i] = clip_uint8(img[i] * f);

    return true;
}