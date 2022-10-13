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

float get_factor();
bool deflicker(uint8_t *img, int w, int h);

#endif
