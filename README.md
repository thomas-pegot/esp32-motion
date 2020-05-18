# Motion detection library (for ESP32cam)


# Introduction

The purpose of this library is to implement robust motion detection algorithm for the ESP32cam or other embedded chip.

The first approach is to compute what is called **optical flow**. 

At the moment I implemented the [Lucas Kanade approach] (https://en.wikipedia.org/wiki/Lucas%E2%80%93Kanade_method).

<div style="width:100%;height:0px;position:relative;padding-bottom:73.077%;"><iframe src="https://streamable.com/e/r1vt77" frameborder="0" width="100%" height="100%" allowfullscreen style="width:100%;height:100%;position:absolute;left:0px;top:0px;overflow:hidden;"></iframe></div>


# TODO : 

1. Filter non relevant vector from optical flow :
    - remove isolated vector (by using a cluster min of vector parameter)
    - remove vector whose direction are too spread compared to the avge vector from cluster.

2. motion detection methods implementation:
    - block matching algorithm
    - Lucas Kanade DoG (Difference of gaussian)
    - KLT approach



# REF:

convolution credits to  http://www.songho.ca/dsp/convolution/convolution.html
