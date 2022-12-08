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
  - [Macros (optional)](#macros-optional)
  - [Example project](#example-project)
  - [TODOs](#todos)
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

## Macros (optional)

In `epzs.c` changing `#define FFMPEG 0` to `1` will use [ffmpeg](https://github.com/FFmpeg/FFmpeg/) version instead of [paper](https://doi.org/10.15406/oajs.2017.01.00002)

In `lucas_kanade_optical_flow.c` changing `#define NOSMOOTH 1` to `0` will enable isotropic smooth causing an increase in latency.

## Example project

 - [ Motion vector stream for testing](https://github.com/thomas-pegot/camera_web_server)
 - [ All in one security camera ](https://github.com/thomas-pegot/ESP32-CAM_Motion)


## TODOs
 - Implement a skip for the first frame in EPZS that use prediction of previous frame. This might the reason why EPZS can have poor result at start.

## Refs
  - convolution credits to  http://www.songho.ca/dsp/convolution/convolution.html
  - EPZS credits to :
    -  https://github.com/FFmpeg/FFmpeg/
    -  [DOI: 10.15406/oajs.2017.01.00002](https://doi.org/10.15406/oajs.2017.01.00002)
