#include "kalman_filter_linear.h"

void kalman_filter_linier
(
		data_t z[3],
		data_t x_est[6],
		bool &read,
		bool &write,
		data_t sat_out[1]
)
{
	//
	const data_t dt = 0.1f;
	const data_t A[6][6] = {
			{1, 0, 0, dt, 0,  0},
			{0, 1, 0, 0,  dt, 0},
			{0, 0, 1, 0,  0,  dt},
			{0, 0, 0, 1,  0,  0},
			{0, 0, 0, 0,  1,  0},
			{0, 0, 0, 0,  0,  1}
	};
	const data_t H[3][6] = {
			{1, 0, 0, 0, 0, 0},
			{0, 1, 0, 0, 0, 0},
			{0, 0, 1, 0, 0, 0}
	};
	const data_t Q[6][6] = {
			{0.0100,0     ,0     ,0     ,0     ,0     },
			{0     ,0.0100,0     ,0     ,0     ,0     },
			{0     ,0     ,0.0100,0     ,0     ,0     },
			{0     ,0     ,0     ,0.0500,0     ,0     },
			{0     ,0     ,0     ,0     ,0.0500,0     },
			{0     ,0     ,0     ,0     ,0     ,0.0500}
	};

	const data_t R[3][3] = {
			{0.5000,0     ,0    },
			{0     ,0.5000,0    },
			{0     ,0     ,1.000},
	};

	static data_t p[6][6];
	static data_t sat = 0;
	static data_t t = 0;
	static data_t x_estR[6];

	data_t x_pred[6];
	data_t P_pred[6][6];
	data_t S[3][3];
	data_t K[6][3];
	data_t y[3];
	data_t temp1[6][6];
	data_t temp2[6][3];

#pragma HLS INTERFACE ap_none port=z
#pragma HLS INTERFACE ap_none port=x_est
#pragma HLS INTERFACE ap_none port=read
#pragma HLS INTERFACE ap_none port=write
#pragma HLS INTERFACE ap_none port=sat_out

#pragma HLS ARRAY_PARTITION variable=z complete dim=1
#pragma HLS ARRAY_PARTITION variable=x_est complete
#pragma HLS ARRAY_PARTITION variable=p complete dim=0
#pragma HLS ARRAY_PARTITION variable=x_pred complete dim=0
#pragma HLS ARRAY_PARTITION variable=P_pred complete dim=0
#pragma HLS ARRAY_PARTITION variable=K complete dim=0
#pragma HLS ARRAY_PARTITION variable=S complete dim=0
#pragma HLS ARRAY_PARTITION variable=y complete dim=0
#pragma HLS ARRAY_PARTITION variable=temp1 complete dim=0
#pragma HLS ARRAY_PARTITION variable=temp2 complete dim=0

	// === Main Kalman loop ===
	read = 0;
	write = 0;
	if(sat == 0)
	{
		for (int i = 0; i < state_dim; i++)
		{
#pragma HLS UNROLL
			for (int j = 0; j < state_dim; j++)
			{
				if (i == j)
					p[i][j] = 10.0f;
				else
					p[i][j] = 0.0f;
			}
		}

		read= 1;
		sat = 1;
	}
	else if(sat == 1)
	{
		for (int i = 0; i < 3; i++)
		{
#pragma HLS PIPELINE II=1
			x_estR[i] = z[i];
			x_estR[i+3] = 0.0f;
		}

		for (int i = 0; i < 6; i++)
		{
			write = 1;
#pragma HLS UNROLL
			x_est[i] = x_estR[i];
		}
		sat = 2;
	}
	else if(sat == 2)
	{
		// Prediction: x_pred = A * x_est(:,t-1)
		for (int i = 0; i < 6; i++)
		{
			data_t sum = 0;
			for (int j = 0; j < 6; j++)
			{
				sum += A[i][j] * x_estR[j];
			}
			x_pred[i] = sum;
		}

		read= 1;
		sat = 3;
	}
	else if(sat == 3)
	{

		// P_pred = A * P * A' + Q
		for (int i = 0; i < 6; i++)
		{
			for (int j = 0; j < 6; j++)
			{
				data_t sum = 0;
				for (int k = 0; k < 6; k++)
				{
					sum += A[i][k] * p[k][j];
				}
				temp1[i][j] = sum;
			}
		}
		//
		for (int i = 0; i < 6; i++)
		{
			for (int j = 0; j < 6; j++)
			{
				data_t sum = 0;
				for (int k = 0; k < 6; k++)
				{
					sum += temp1[i][k] * A[j][k]; // A'
				}
				P_pred[i][j] = sum + Q[i][j];
			}
		}

		// S = H * P_pred * H' + R
		data_t tempS[3][6];
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 6; j++)
			{
				data_t sum = 0;
				for (int k = 0; k < 6; k++)
				{
					sum += H[i][k] * P_pred[k][j];
				}
				tempS[i][j] = sum;
			}
		}
		//
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				data_t sum = 0;
				for (int k = 0; k < 6; k++)
				{
					sum += tempS[i][k] * H[j][k]; // H'
				}
				S[i][j] = sum + R[i][j];
			}
		}

		// Compute K = P_pred * H' / S (simplified inverse for 3x3)
		// We'll assume S is diagonal (approximation)
		for (int i = 0; i < 6; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				data_t sum = 0;
				for (int k = 0; k < 6; k++)
				{
					sum += P_pred[i][k] * H[j][k]; // H'
				}
				if (fabsf(S[j][j]) > 1e-6)
					K[i][j] = sum / S[j][j];
				else
					K[i][j] = 0;
			}
		}
		// Innovation y = z_t - H * x_pred
		for (int i = 0; i < 3; i++)
		{
			data_t sum = 0;
			for (int j = 0; j < 6; j++)
			{
				sum += H[i][j] * x_pred[j];
			}
			y[i] = z[i] - sum;
		}

		// State update: x_upd = x_pred + K * y
		data_t x_upd[6];
		for (int i = 0; i < 6; i++)
		{
			data_t sum = 0;
			for (int j = 0; j < 3; j++)
			{
				sum += K[i][j] * y[j];
			}
			x_upd[i] = x_pred[i] + sum;
		}

		// P = (I - K*H)*P_pred
		data_t KH[6][6];
		for (int i = 0; i < 6; i++)
		{
			for (int j = 0; j < 6; j++)
			{
				data_t sum = 0;
				for (int k = 0; k < 3; k++)
				{
					sum += K[i][k] * H[k][j];
				}
				KH[i][j] = sum;
			}
		}
		//
		data_t KH1[6][6];
		for (int i = 0; i < 6; i++)
		{
			for (int j = 0; j < 6; j++)
			{
				if(i==j)KH1[i][j] = 1.0f - KH[i][j];
				else KH1[i][j] = 0.0f - KH[i][j];
			}
		}
		//
		loop_p1 : for (int i = 0; i < 6; i++)
		{
			loop_p2 : for (int j = 0; j < 6; j++)
			{
#pragma HLS PIPELINE II = 3
				data_t sum = 0;
				for (int k = 0; k < 6; k++)
				{
					sum += (KH1[i][k]) * P_pred[k][j];
				}
				p[i][j] = sum;
			}
		}

		// result
		loop_res1 : for (int i = 0; i < 6; i++)
		{
#pragma HLS PIPELINE II = 2
			x_estR[i] = x_upd[i];
		}

		write = 1;
		loop_res2 : for (int i = 0; i < 6; i++)
		{
//#pragma HLS UNROLL
			x_est[i] = x_estR[i];
		}
		//

		t++;
		if (t == TMAX-1)sat = 4;
		else sat = 2;
	}
	sat_out[0] = sat;
}
