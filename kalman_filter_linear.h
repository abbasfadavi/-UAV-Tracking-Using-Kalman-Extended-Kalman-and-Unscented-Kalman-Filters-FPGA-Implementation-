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

#define TMAX 100
#define state_dim 6

typedef float data_t;

void kalman_filter_linier
(
    data_t z[3],
    data_t x_est[6],
	bool &read,
	bool &write,
	data_t sat_out[1]
);


