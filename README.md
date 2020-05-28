<H1> Motion detection library (for ESP32cam) </H1>


## 0.1. Introduction

The purpose of this library is to implement **robust motion detection** algorithm for the **ESP32cam** and other embedded chip.



## 0.2. Basic usage

### 0.2.1. Declaration and initialisation :

First thing first create a _motion estimation context_ :

```c
MotionEstContext me_ctx = {.method = LK_OPTICAL_FLOW,    // algorithm used
                           .width  = 240,  .height = 240 // size of your image
                           };
```

Next allocate motion vectors:

```c
if(!alloc_mv(&mec_ctx))
    Serial.println("Failed to allocate vectors!");
```

Other way :

```c
MotionEstContext me_ctx;
init_context(        &ctx,
          LK_OPTICAL_FLOW,  // algo used 
                   mbSize,  // block size for block matching algo
             search_param,  // search parameter value for block matching algo
                 240, 240); // images size

```

### 0.2.2. Estimate motion :

Now you can call `motion_estimation` method and pass current and previous images buffer.

```c
if(!motion_estimation(&ctx, (uint8_t *)img_prev, (uint8_t *)img_cur))
    Serial.println("motion estimtion failed!")!
```

Now motion vectors will be stored in `me_ctx.mv_table[0]` with the maximum being `me_ctx.max`.

Note : `mv_table` acts as a FIFO which means each time you perform an estimation it will push the FIFO :
```mermaid
graph LR;
motion_estimation --> mv_table0 --> mv_table1 --> mv_table2 --> NULL;

```

### 0.2.3. Free memory :

```c
uninit(&ctx);
```


## 0.3. More in depth 

### 0.3.1. [Block matching (Adaptative Rood Pattern Search)](https://en.wikipedia.org/wiki/Block-matching_algorithm#cite_note-8)

 - header :
    ```c
    /** @brief Computes motion vectors using Adaptive Rood Pattern Search method
    * 
    * @param imgP   : image of which we want to find motion vectors 
    * @param imgI   : reference image 
    * @param w      : width of image
    * @param h      : height of image
    * @param mbSize : Size of the macroblock (mbSize, mbSize)
    * @param p      : Search parameter
    * @param zmp_T  : Zero-Motion Prejudgement threshold enable if set superior at 0. 
    * improve performance at cost of precision if wrong thresold value.
    * 
    * @param [output] motionVect :  motion vector for each integral macroblock in imgP.  size = w * h/mbSize²
    * @param [output] max_mag2 :  max mag2 in motionVect 
    * @return success/fail
    **/
    bool motionEstARPS(const uint8_t *imgP, const uint8_t *imgI, size_t w, size_t h, size_t mbSize, int p, MotionVector16_t *MotionVect, int zmp_T, int *max_mag2)
    ```

 - example :
      ```c
    // image_buf image you capture from a stream or else
    int N = w * h;
    int max;
    uint8_t *image_buf_next = calloc(N, 1); 
    uint8_t *image_motion = malloc(N); 
    MotionVector16_t *vecor_motion = (MotionVector16_t*)calloc(N, sizeof(MotionVector16_t));
    while(1) {
            capture(img_buf); // Your algorithm
            if(!motionEstARPS(image_buf, image_buf_next, w, h, 8, 6, vector_motion, 256, &max)) {
                Serial.printf("motion failed!");
                break;
            }
            memcpy(image_buf_next, image_buf, N);
            display(vector_motion);
            Serial.printf("max motion: %u \n", a);
    }
    free(image_buf_next);
    free(image_buf);
    free(image_motion);
    ```



### 0.3.2. [Lucas Kanade algorithm](https://en.wikipedia.org/wiki/Lucas%E2%80%93Kanade_method)

#### 0.3.2.1. Simple case

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

### 0.3.3. More control :

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

## 0.4. TODOs 

 - [ ]  Add function to filter non relevant vector from optical flow :
    - [ ] remove isolated vector (by using a cluster min of vector parameter)
    - [ ] remove vector whose direction are too spread compared to the avge vector from cluster.

 - [ ] Alternate motion detection methods implementation:
    - [ ] block matching algorithm
      - [x] Adaptative Rood Pattern Search
      - [ ] Enhanced Predictive Zonal Search
    - [ ] Lucas Kanade DoG (Difference of gaussian)




## 0.5. Refs

convolution credits to  http://www.songho.ca/dsp/convolution/convolution.html
