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
#define STATE_DIM 6
#define MEAS_DIM 3
#define T_MAX 100
#define ALPHA 0.05f
#define BETA 2.0f
#define KAPPA 0.0f
#define REG 1e-6f

typedef float data_t;

void kalman_filter_unextended
(
    data_t z_in[MEAS_DIM][T_MAX],
    data_t x_est[STATE_DIM][T_MAX]
);


