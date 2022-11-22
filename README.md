# Motion estimation lib (ESP32cam)

   More information in [Doxygen documentation](https://thomas-pegot.github.io/esp32-motion)

[![Gitpod Ready-to-Code](https://img.shields.io/badge/Gitpod-Ready--to--Code-blue?logo=gitpod)](https://gitpod.io/#https://github.com/thomas-pegot/camera_web_server)
<H3>Contents :</H3>

- [Motion estimation lib (ESP32cam)](#motion-estimation-lib-esp32cam)
  - [Introduction](#introduction)
  - [Basic usage](#basic-usage)
    - [Declaration and initialisation :](#declaration-and-initialisation-)
    - [Estimate motion :](#estimate-motion-)
    - [Free memory :](#free-memory-)
  - [Example project](#example-project)
  - [More in depth (_depreciated_)](#more-in-depth-depreciated)
    - [Block matching (Adaptative Rood Pattern Search) (_depreciated_)](#block-matching-adaptative-rood-pattern-search-depreciated)
    - [Lucas Kanade algorithm (_depreciated_)](#lucas-kanade-algorithm-depreciated)
      - [Simple case (_depreciated_)](#simple-case-depreciated)
    - [More control (_depreciated_)](#more-control-depreciated)
  - [Refs](#refs)


## Introduction

The purpose of this library is to implement **robust motion estimation** algorithm for the **ESP32cam** and other embedded chip.
_Motion estimation_ is the process of finding motion vectors that define the translation from one image to another. This can be resolved by differennt approach:

 - block matching algorithms ( ES, TSS, ARPS, EPZS)
 - optical flow (Lucas-Kanade, Horn-Schunk)
 - pixel recursive algorithm (RANSAC)
 - phase correlation (FFT based)

At the moment, I have implemented Lucas-kanade, ARPS and EPZS (FFMPEG + AVC/MPEG4 paper).


## Basic usage

### Declaration and initialisation :

First thing first create a [_motion estimation context_](https://thomas-pegot.github.io/esp32-motion/struct_motion_est_context.html) :

```c
MotionEstContext me_ctx = {.method = LK_OPTICAL_FLOW,    // algorithm used
                           .width  = 96,  .height = 96 // size of your image
                           };

MotionEstContext me_ctx2 = {
.method = BLOCK_MATCHING_ARPS,  // algo used 
                  .mbSize = 8,  // block size for block matching algo
            .search_param = 7,  // search parameter value for block matching algo
  .width = 640, .height = 480); // images size
                            }
```

table of correspondance :

| macro  | val  |  function called  |
|---|---|---|
|LK_OPTICAL_FLOW_8BIT| 0 |lucas kanade (out 8-bit uchar)|
|LK_OPTICAL_FLOW | 1 | lucas kanade (out 16-bit vector)|
|BLOCK_MATCHING_ARPS| 2 | ARPS (out 16-bit vector)|
|BLOCK_MATCHING_EPZS| 3 | EPZS (out 16-bit vector)|

Next <a href="https://thomas-pegot.github.io/esp32-motion/motion_8h.html#a307035191f24ff24a02add340d8b4efa">allocate motion</a> vectors:
```c
init_context(&me_ctx);
```

  
### Estimate motion :

Now you can call [`motion_estimation`](https://thomas-pegot.github.io/esp32-motion/motion_8c.html#a8ba35bcbf89a11452927cc1ce2710edd) method and pass current and previous images buffer.

```c
if(!motion_estimation(&me_ctx, (uint8_t *)img_prev, (uint8_t *)img_cur))
    Serial.println("motion estimtion failed!")!
```

Now motion vectors will be stored in `me_ctx.mv_table[0]` with the maximum being `me_ctx.max`.

Note : in case of EZPS algorithm, `mv_table` acts as a FIFO which means each time you perform an estimation it will push the FIFO :
```mermaid
graph LR;
motion_estimation --> mv_table0 --> mv_table1 --> mv_table2 --> NULL;

```

EZPS algorithm need previous motion vectors as a way of prediction to the next generated.


### Free memory :

```c
uninit(&me_ctx);
```
## Example project

 - [ Motion vector stream for testing](https://github.com/thomas-pegot/camera_web_server)
 - [ All in one security camera ](https://github.com/thomas-pegot/ESP32-CAM_Motion)


---

```diff 
- Below is depreciated
```

## More in depth (_depreciated_)

### Block matching (Adaptative Rood Pattern Search) (_depreciated_)

 [header:](https://thomas-pegot.github.io/esp32-motion/block__matching_8c.html#a58f37a2a134b9ff537305104c3f15495)

  ```c
  bool motionEstARPS(const uint8_t *imgP, const uint8_t *imgI, size_t w, size_t h, size_t mbSize, int p, MotionVector16_t *MotionVect, int zmp_T, int *max_mag2)
  ```

  example :

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



### Lucas Kanade algorithm (_depreciated_)

#### Simple case (_depreciated_)

[header:](https://thomas-pegot.github.io/esp32-motion/lucas__kanade__opitcal__flow_8c.html#a22663424a50db0dd70de24dd8b176f39)

  ```c
  bool LK_optical_flow8(const uint8_t *src1, const uint8_t *src2, uint8_t *V, int w, int h);
  ```
   
 example : 

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

### More control (_depreciated_)

We can get more detailed output by using a motion vector struct composed of `V=(vx, vy)` motion vector and `mag2` the squared magnitude:

  ```c
   typedef struct {
      int16_t vx,
              vy;
      uint16_t mag2; // squared magnitude
   } MotionVector16_t;
  ```
  header:

  ```c
  bool LK_optical_flow(const uint8_t *src1, const uint8_t *src2, MotionVector16_t *v, int w, int h);
  ```

  example :

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
  free(vector);
  free(image_buf);
  free(image_buf_next);
  ```

## Refs
  - convolution credits to  http://www.songho.ca/dsp/convolution/convolution.html
  - EPZS credits to :
    -  https://github.com/FFmpeg/FFmpeg/
    -  [DOI: 10.15406/oajs.2017.01.00002](https://doi.org/10.15406/oajs.2017.01.00002)
