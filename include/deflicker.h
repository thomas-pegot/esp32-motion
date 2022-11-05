/** @file deflicker.h
*  Took from ffmpeg but simplified
*/

#ifndef DEFLICKER_H
#define DEFLICKER_H

#define MAXSIZE 10

typedef unsigned char uint8_t;

/** @struct queue
*   @brief simple queue 
*/
typedef struct queue {
    float brightness[MAXSIZE];  ///< rolling brightness
    int available;
} queue_t;  

/** @brief calculate brightness ratio related to previous brightness
*   @return ratio (float)
*/
float get_factor();

/** @brief perform deflickering
*   @param img uint8 pointer to image
*   @param w width
*   @param h height
*   @return true if deflickering else return false (not enough image queued)
*/
bool deflicker(uint8_t *img, int w, int h);

#endif
