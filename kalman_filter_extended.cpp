#include "kalman_filter_extended.h"

//
void kalman_filter_extended
(
		const data_t z[3][TMAX],
		data_t x_est[6][TMAX]
)
{
#pragma HLS INTERFACE ap_none port=z
#pragma HLS INTERFACE ap_none port=x_est
#pragma HLS INTERFACE ap_ctrl_none port=return

#pragma HLS ARRAY_PARTITION variable=z     complete
#pragma HLS ARRAY_PARTITION variable=x_est complete

	// Constants
	const int STATE_DIM = 6;
	const int MEAS_DIM  = 3;
	const data_t dt = 0.1f;
	const data_t Q_pos = 0.01f;
	const data_t Q_vel = 0.001f;
	const data_t R[3][3] = {
			{0.5000,0     ,0     },
			{0     ,0.0349,0     },
			{0     ,0     ,0.0349},
	};
	const data_t F[6][6] = {
			{1.0000,0     ,0     ,0.1000,0     ,0     },
			{0     ,1.0000,0     ,0     ,0.1000,0     },
			{0     ,0     ,1.0000,0     ,0     ,0.1000},
			{0     ,0     ,0     ,1.0000,0     ,0     },
			{0     ,0     ,0     ,     0,1.0000,0     },
			{0     ,0     ,0     ,     0,0     ,1.0000},
	};
	const data_t Q[6][6] = {
			{0.0100,0     ,0     ,0     ,0     ,0     },
			{0     ,0.0100,0     ,0     ,0     ,0     },
			{0     ,0     ,0.0100,0     ,0     ,0     },
			{0     ,0     ,0     ,0.0010,0     ,0     },
			{0     ,0     ,0     ,0     ,0.0010,0     },
			{0     ,0     ,0     ,0     ,0     ,0.0010}
	};
	static data_t P_est[6][6];

#pragma HLS ARRAY_PARTITION variable=R     complete
#pragma HLS ARRAY_PARTITION variable=F     complete
#pragma HLS ARRAY_PARTITION variable=Q     complete
#pragma HLS ARRAY_PARTITION variable=P_est complete

	// Temporary arrays used per time step (fully partitioned)
	data_t x_pred[6];
	data_t P_pred[6][6];
	data_t H[3][6];
	data_t z_pred[3];
	data_t y_innov[3];
	data_t S[3][3];
	data_t invS[3][3];
	data_t K[6][3];
	data_t tmp1[6][6];
	data_t tmp2[6][3];

#pragma HLS ARRAY_PARTITION variable=x_pred  complete dim=0
#pragma HLS ARRAY_PARTITION variable=P_pred  complete dim=0
#pragma HLS ARRAY_PARTITION variable=H       complete dim=0
#pragma HLS ARRAY_PARTITION variable=z_pred  complete dim=0
#pragma HLS ARRAY_PARTITION variable=y_innov complete dim=0
#pragma HLS ARRAY_PARTITION variable=S       complete dim=0
#pragma HLS ARRAY_PARTITION variable=invS    complete dim=0
#pragma HLS ARRAY_PARTITION variable=K       complete dim=0
#pragma HLS ARRAY_PARTITION variable=tmp1    complete dim=0
#pragma HLS ARRAY_PARTITION variable=tmp2    complete dim=0

	// Main EKF loop
	loop_main : for (int t = 0; t < TMAX; t++)
	{
		if(t == 0)
		{
			x_est[2][0] = 10.0f;;
			x_est[3][0] = 1.0f;

			P_est[0][0] = 10.0f;
			P_est[1][1] = 10.0f;
			P_est[2][2] = 10.0f;
			P_est[3][3] = 10.0f;
			P_est[4][4] = 10.0f;
			P_est[5][5] = 10.0f;
		}
		else
		{
			// ---------- Prediction ----------
			// x_pred = F * x_est(:, t-1)
			for (int i = 0; i < 6; i++)
			{
				data_t sum = 0.0f;
				for (int j = 0; j < 6; j++)
				{
#pragma HLS PIPELINE II = 1
					sum += F[i][j] * x_est[j][t-1];
				}
				x_pred[i] = sum;

			}

			// P_pred = F * P * F' + Q
			// tmp1 = F * P
			for (int i = 0; i < 6; i++)
			{
				for (int j = 0; j < 6; j++)
				{
					data_t sum = 0.0f;
					for (int k = 0; k < 6; k++)
					{
#pragma HLS PIPELINE II = 1
						sum += F[i][k] * P_est[k][j];
					}
					tmp1[i][j] = sum;
				}
			}
			// P_pred = tmp1 * F' + Q
			for (int i = 0; i < 6; i++)
			{
				for (int j = 0; j < 6; j++)
				{
					data_t sum = 0.0f;
					for (int k = 0; k < 6; k++)
					{
#pragma HLS PIPELINE II = 1
						sum += tmp1[i][k] * F[j][k]; // F'
					}
					P_pred[i][j] = sum + Q[i][j];
				}
			}

			// ---------- Measurement prediction (nonlinear) ----------
			// Build h(x_pred): range, azimuth, elevation
			{
				data_t px = x_pred[0];
				data_t py = x_pred[1];
				data_t pz = x_pred[2];

				data_t r = sqrtf(px*px + py*py + pz*pz);
				data_t rho_xy = sqrtf(px*px + py*py);

				// avoid zero divisions
				if (r < 1e-6f) r = 1e-6f;
				if (rho_xy < 1e-6f) rho_xy = 1e-6f;

				z_pred[0] = r;
				z_pred[1] = atan2f(py, px);
				z_pred[2] = atan2f(pz, rho_xy);
			}

			// ---------- Jacobian H (3x6) ----------
			{
				data_t px = x_pred[0];
				data_t py = x_pred[1];
				data_t pz = x_pred[2];
				data_t r2 = px*px + py*py + pz*pz;
				data_t r = sqrtf(r2);
				data_t rho2 = px*px + py*py;
				data_t rho = sqrtf(rho2);

				if (r < 1e-6f) r = 1e-6f;
				if (rho < 1e-6f) rho = 1e-6f;

				// dr/dx, dr/dy, dr/dz
				data_t drdx = px / r;
				data_t drdy = py / r;
				data_t drdz = pz / r;

				// daz/dx, daz/dy, daz/dz
				data_t dazdx = -py / (rho2);
				data_t dazdy =  px / (rho2);
				data_t dazdz = 0.0f;

				// del/dx, del/dy, del/dz
				// del = atan2(pz, rho)
				// derivatives derived analytically
				data_t del_dx = - (px * pz) / (r2 * rho);
				data_t del_dy = - (py * pz) / (r2 * rho);
				data_t del_dz =   rho / r2;

				// Fill H (3x6)
				loop_H : for (int j = 0; j < 6; j++)
				{
#pragma HLS UNROLL
					H[0][j] = 0.0f;
					H[1][j] = 0.0f;
					H[2][j] = 0.0f;
				}
				H[0][0] = drdx;
				H[0][1] = drdy;
				H[0][2] = drdz;
				H[1][0] = dazdx;
				H[1][1] = dazdy;
				H[1][2] = dazdz;
				H[2][0] = del_dx;
				H[2][1] = del_dy;
				H[2][2] = del_dz;
				// velocity derivatives are zero (measurements depend on position only)
			}

			// ---------- Innovation y = z(:,t) - z_pred (wrap angles) ----------
			loop_Innovation : for (int i = 0; i < 3; i++)
			{
#pragma HLS PIPELINE II = 1
				y_innov[i] = z[i][t-1] - z_pred[i];

			}
			// wrap azimuth and elevation to [-pi,pi]
			// simple wrap
			if (y_innov[1] >  M_PI) y_innov[1] -= 2.0f * M_PI;
			if (y_innov[1] < -M_PI) y_innov[1] += 2.0f * M_PI;
			if (y_innov[2] >  M_PI) y_innov[2] -= 2.0f * M_PI;
			if (y_innov[2] < -M_PI) y_innov[2] += 2.0f * M_PI;

			// ---------- Innovation covariance S = H*P_pred*H' + R (3x3) ----------
			// tmp2 = H * P_pred  (3x6)
			loop_s1 : for (int i = 0; i < 3; i++)
			{
				loop_s2 : for (int j = 0; j < 6; j++)
				{
					data_t sum = 0.0f;
					loop_s3 : for (int k = 0; k < 6; k++)
					{
#pragma HLS PIPELINE II = 1
						sum += H[i][k] * P_pred[k][j];
					}
					tmp2[j][i] = sum; // store transposed for later use
				}
			}

			// compute S = H * P_pred * H' + R  (3x3)
			loop_s4 : for (int i = 0; i < 3; i++)
			{
				loop_s5 : for (int j = 0; j < 3; j++)
				{
					data_t sum = 0.0f;
					loop_s6 : for (int k = 0; k < 6; k++)
					{
					#pragma HLS PIPELINE II = 1
						// tmp2[k][i] is (H*P_pred)_{i,k} transposed; access accordingly:
						sum += tmp2[k][i] * H[j][k];
					}
					S[i][j] = sum + R[i][j];
				}
			}

			// ---------- invert S (3x3) => invS ----------
			// compute determinant
			data_t detS = S[0][0] * (S[1][1]*S[2][2] - S[1][2]*S[2][1])
                    		- S[0][1] * (S[1][0]*S[2][2] - S[1][2]*S[2][0])
							+ S[0][2] * (S[1][0]*S[2][1] - S[1][1]*S[2][0]);

			// If det is too small, regularize
			if (fabsf(detS) < 1e-6f)
			{
				// add small diagonal regularization
				S[0][0] += 1e-3f;
				S[1][1] += 1e-3f;
				S[2][2] += 1e-3f;
				detS = S[0][0] * (S[1][1]*S[2][2] - S[1][2]*S[2][1])
                		 - S[0][1] * (S[1][0]*S[2][2] - S[1][2]*S[2][0])
						 + S[0][2] * (S[1][0]*S[2][1] - S[1][1]*S[2][0]);
				if (fabsf(detS) < 1e-9f) detS = 1e-9f; // avoid zero
			}

			// adjugate / det
			invS[0][0] =  (S[1][1]*S[2][2] - S[1][2]*S[2][1]) / detS;
			invS[0][1] = -(S[0][1]*S[2][2] - S[0][2]*S[2][1]) / detS;
			invS[0][2] =  (S[0][1]*S[1][2] - S[0][2]*S[1][1]) / detS;
			invS[1][0] = -(S[1][0]*S[2][2] - S[1][2]*S[2][0]) / detS;
			invS[1][1] =  (S[0][0]*S[2][2] - S[0][2]*S[2][0]) / detS;
			invS[1][2] = -(S[0][0]*S[1][2] - S[0][2]*S[1][0]) / detS;
			invS[2][0] =  (S[1][0]*S[2][1] - S[1][1]*S[2][0]) / detS;
			invS[2][1] = -(S[0][0]*S[2][1] - S[0][1]*S[2][0]) / detS;
			invS[2][2] =  (S[0][0]*S[1][1] - S[0][1]*S[1][0]) / detS;

			// ---------- Kalman gain K = P_pred * H' * invS  (6x3) ----------
			// first compute P_pred * H'  (6x3)
			loop_pred1 : for (int i = 0; i < 6; i++)
			{
				loop_pred2 : for (int j = 0; j < 3; j++)
				{
					data_t sum = 0.0f;
					loop_pred3 : for (int k = 0; k < 6; k++)
					{
#pragma HLS PIPELINE II = 1
						sum += P_pred[i][k] * H[j][k]; // H' used
					}
					tmp2[i][j] = sum; // reuse tmp2 as (6x3)
				}
			}
			// K = tmp2 * invS  (6x3)
			loop_k1 : for (int i = 0; i < 6; i++)
			{
				loop_k2 : for (int j = 0; j < 3; j++)
				{
					data_t sum = 0.0f;
					loop_k3 : for (int k = 0; k < 3; k++)
					{
#pragma HLS PIPELINE II = 1
						sum += tmp2[i][k] * invS[k][j];
					}
					K[i][j] = sum;
				}
			}

			// ---------- Update step ----------
			// x_est(:,t) = x_pred + K * y_innov
			loop_est1 : for (int i = 0; i < 6; i++)
			{
				data_t sum = 0.0f;
				loop_est2 : for (int j = 0; j < 3; j++)
				{
#pragma HLS PIPELINE II = 1
					sum += K[i][j] * y_innov[j];
				}
				x_est[i][t] = x_pred[i] + sum;
			}

			// P = (I - K*H) * P_pred
			data_t KH[6][6];
#pragma HLS ARRAY_PARTITION variable=KH complete dim=0
			loop_KH1 : for (int i = 0; i < 6; i++)
			{
				loop_KH2 : for (int j = 0; j < 6; j++)
				{
					data_t sum = 0.0f;
					loop_KH3 : for (int k = 0; k < 3; k++)
					{
#pragma HLS PIPELINE II = 1
						sum += K[i][k] * H[k][j];
					}
					KH[i][j] = sum;
				}
			}

			for (int i = 0; i < 6; i++)
				for (int j = 0; j < 6; j++)
				{
					if(i == j)KH[i][j] = 1.0f - KH[i][j];
					else KH[i][j] = 0.0f - KH[i][j];
				}

			loop_pest1 : for (int i = 0; i < 6; i++)
			{
				loop_pest2 : for (int j = 0; j < 6; j++)
				{
					data_t sum = 0.0f;
					loop_pest3 : for (int k = 0; k < 6; k++)
					{
#pragma HLS PIPELINE II = 1
						sum += KH[i][k] * P_pred[k][j];
					}
					P_est[i][j] = sum;
				}
			}
		}
	} // end time loop
}
