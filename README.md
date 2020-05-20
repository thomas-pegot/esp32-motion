# Motion detection library (for ESP32cam)


# Introduction

The purpose of this library is to implement **robust motion detection** algorithm for the ESP32cam or other embedded chip.

The first approach is to compute what is called **optical flow**. 

At the moment I implemented the [Lucas Kanade approach] (https://en.wikipedia.org/wiki/Lucas%E2%80%93Kanade_method).


# Usage:

## Lucas Kanade algorithm

### Simple case

 - header :
  ```c
  /** @brief Implement LK optical flow 8bit version Magnitude
 * @param src1 pointer to grayscale buffer image instant t. 
 * @param src2 pointer to grayscale buffer image instant t+1.
 * @param out [out] image 8bit depth output of squared magnitude
 * @return True if success False if failed somewhere*/
  bool LK_optical_flow8(const uint8_t *src1, const uint8_t *src2, uint8_t *V, int w, int h);
  ```
  - example : 

  ```c
  // image_buf image you capture from a stream or else
  int N = w * h;
  uint8_t *image_buf_next = calloc(N, 1); 
  uint8_t *image_motion = malloc(N); 
  while(1) {
        capture(img_buf); // Your algorithm
        if(!LK_optical_flow8(image_buf, image_buf_next, image_motion, w, h)) {
            Serial.printf("motion failed!");
            break;
        }
        memcpy(image_buf_next, image_buf, N);
        display(image_motion);
  }
  free(image_buf_next);
  free(image_buf);
  free(image_motion);
 ```

### More control :

We can get more detailed output by using a motion vector struct composed of `V=(vx, vy)` motion vector and `mag2` the squared magnitude:

  ```c
 typedef struct {
    int16_t vx,
            vy;
    uint16_t mag2; // squared magnitude
} MotionVector16_t;
```
  - headers:
  ```c
  /** @brief Implement LK optical flow source from wiki and matlab:
 * https://en.wikipedia.org/wiki/Lucas%E2%80%93Kanade_method
 * @param src1 pointer to grayscale buffer image instant t. 
 * @param src2 pointer to grayscale buffer image instant t+1.
 * @param V [out] vector (vx, vy) and squared magnitude
 * @return True if success False if failed somewhere*/
  bool LK_optical_flow(const uint8_t *src1, const uint8_t *src2, MotionVector16_t *v, int w, int h);
  ```

  - example :

  ```c
  // image_buf image you capture from a stream or else
  int N = w * h;
  uint8_t *image_buf_next = calloc(N, 1); 
  uint8_t *image_motion = malloc(N); 
  MotionVector16_t* vector = (MotionVector16_t*) calloc(count, sizeof(MotionVector16_t));

  float max = 0;
  int half_window = (int)WINDOW / 2.0; // WINDOW >> 1

  while(1) {
        capture(img_buf); // Your algorithm
        if(!LK_optical_flow(image_buf, image_buf_next, vector, w, h)) {
            Serial.printf("motion failed!");
            break;
        }
        memcpy(image_buf_next, image_buf, N);

        // Extract max squared magnitude
        mv = &vector[half_window * w + half_window];
        for(int i = h - WINDOW; i--; ) { // same as for(int i = half_window; i < h - half_window; i++) but faster
            for(int j = half_window; j < w - half_window; j++, mv++) {
                if (mv->mag2 > max)  
                    max = mv->mag2;
            }
            mv += WINDOW;
        }
        Serial.printf("max motion = %u", (short)max);
  }
  free(vector)
  free(image_buf);
  free(image_buf_next);
 ```

# TODOs 

1. Add function to filter non relevant vector from optical flow :
    - remove isolated vector (by using a cluster min of vector parameter)
    - remove vector whose direction are too spread compared to the avge vector from cluster.

2. Alternate motion detection methods implementation:
    - block matching algorithm
    - Lucas Kanade DoG (Difference of gaussian)
    - KLT approach


# Refs

convolution credits to  http://www.songho.ca/dsp/convolution/convolution.html
