#pragma once

#include <ap_fixed.h>
#include <hls_math.h>
#include <hls_vector.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <stdio.h>
#include "hls_stream.h"
#include "ap_int.h"
#include <math.h>

#define TMAX 100

typedef float data_t;

void kalman_filter_extended
(
    const data_t z[3][TMAX],        // measurements (range, azimuth, elevation)
    data_t x_est[6][TMAX]           // output estimates (6 x T)
);


