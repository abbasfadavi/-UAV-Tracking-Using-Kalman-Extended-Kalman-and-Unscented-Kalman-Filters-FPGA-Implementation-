#include "kalman_filter_unextended.h"

void kalman_filter_unextended
(
		data_t z_in[MEAS_DIM][T_MAX],
		data_t x_est[STATE_DIM][T_MAX]
)
{
#pragma HLS ARRAY_PARTITION variable=z_in complete
#pragma HLS ARRAY_PARTITION variable=x_est complete

	const int n = STATE_DIM;
	const int m = MEAS_DIM;
	const int L = 2*n + 1;
	const data_t lambda = ALPHA*ALPHA*(n+KAPPA)-n;
	const data_t c = n + lambda;
	const data_t Q_scale = 10.00f;
	const data_t dt = 0.1f;
	const data_t R[3][3] = {
			{0.5000,0     ,0     },
			{0     ,0.0349,0     },
			{0     ,0     ,0.0349},
	};
	const data_t Q[STATE_DIM][STATE_DIM] = {
			{10.0f, 00.0f, 00.0f, 00.0f, 00.0f, 00.0f},
			{00.0f, 10.0f, 00.0f, 00.0f, 00.0f, 00.0f},
			{00.0f, 00.0f, 10.0f, 00.0f, 00.0f, 00.0f},
			{00.0f, 00.0f, 00.0f, 10.0f, 00.0f, 00.0f},
			{00.0f, 00.0f, 00.0f, 00.0f, 10.0f, 00.0f},
			{00.0f, 00.0f, 00.0f, 00.0f, 00.0f, 10.0f}
	};

	const data_t Wm[13] = {-399.0f,33.33333f,33.33333f,33.33333f,33.33333f,33.33333f,33.33333f,33.33333f,33.33333f,33.33333f,33.33333f,33.33333f,33.33333f};
	const data_t Wc[13] = {-396.0025f,33.33333f,33.33333f,33.33333f,33.33333f,33.33333f,33.33333f,33.33333f,33.33333f,33.33333f,33.33333f,33.33333f,33.33333f};

	data_t x[STATE_DIM];
	data_t P[STATE_DIM][STATE_DIM] = {
			{10.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
			{0.0f, 10.0f, 0.0f, 0.0f, 0.0f, 0.0f},
			{0.0f, 0.0f, 10.0f, 0.0f, 0.0f, 0.0f},
			{0.0f, 0.0f, 0.0f, 10.0f, 0.0f, 0.0f},
			{0.0f, 0.0f, 0.0f, 0.0f, 10.0f, 0.0f},
			{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 10.0f}
	};

	data_t Xsigma[STATE_DIM][L];
	data_t Xsigma_pred[STATE_DIM][L];
	data_t Zsigma[MEAS_DIM][L];

#pragma HLS ARRAY_PARTITION variable=x complete
#pragma HLS ARRAY_PARTITION variable=Xsigma complete
#pragma HLS ARRAY_PARTITION variable=Xsigma_pred complete
#pragma HLS ARRAY_PARTITION variable=Zsigma complete

	// main time loop
	TIME_LOOP: for(int t = 0; t < TMAX; t++)
	{
		if(t == 0)
		{
			INIT_LOOP: for(int i=0;i<STATE_DIM;i++)
			{
#pragma HLS PIPELINE II = 1
				if(i<3) x[i]=z_in[i][0];
				else x[i]=0.0f;
				x_est[i][0]=x[i];
			}
		}
		else
		{
			// --- Sigma points ---
			data_t S[STATE_DIM][STATE_DIM]={0};
#pragma HLS ARRAY_PARTITION variable=S complete
			CHOL_LOOP: for(int i=0;i<n;i++)
			{
#pragma HLS PIPELINE II = 1
				S[i][i] = hls::sqrtf(c*(P[i][i]+REG));
			}

			SIGMA_LOOP: for(int i=0;i<n;i++)
			{
				Xsigma[i][0]=x[i];
				SIGMA_INNER: for(int j=0;j<n;j++)
				{
#pragma HLS PIPELINE II = 1
					Xsigma[i][j+1  ]=x[i]+S[i][j];
					Xsigma[i][j+1+n]=x[i]-S[i][j];
				}
			}

			// --- Prediction ---
			PRED_LOOP_K: for(int k=0;k<L;k++)
			{
				PRED_LOOP_I: for(int i=0;i<STATE_DIM;i++)
				{
#pragma HLS PIPELINE
					if(i<3)
						Xsigma_pred[i][k] = Xsigma[i][k] + dt*Xsigma[i+3][k];
					else
						Xsigma_pred[i][k] = Xsigma[i][k];
				}
			}

			// predicted mean
			data_t x_pred[STATE_DIM]={0};
			MEAN_LOOP_I: for(int i=0;i<n;i++)
				MEAN_LOOP_K: for(int k=0;k<L;k++)
				{
#pragma HLS PIPELINE II = 1
					x_pred[i]+=Wm[k]*Xsigma_pred[i][k];
				}

			// predicted covariance
			data_t P_pred[STATE_DIM][STATE_DIM]={0};
			COV_LOOP_I: for(int i=0;i<n;i++)
				COV_LOOP_J: for(int j=0;j<n;j++)
					COV_LOOP_K: for(int k=0;k<L;k++)
					{
#pragma HLS PIPELINE II = 2
						data_t dx_i = Xsigma_pred[i][k]-x_pred[i];
						data_t dx_j = Xsigma_pred[j][k]-x_pred[j];
						P_pred[i][j]+=Wc[k]*dx_i*dx_j;
					}

				ADD_Q_LOOP_I: for(int i=0;i<n;i++)
					ADD_Q_LOOP_J: for(int j=0;j<n;j++)
						P_pred[i][j]+=Q[i][j];

				// Measurement prediction
				MEAS_LOOP_K: for(int k=0;k<L;k++)
				{
					MEAS_LOOP_I: for(int i=0;i<m;i++)
					{
#pragma HLS PIPELINE II = 1
						Zsigma[i][k] = Xsigma_pred[i][k];
					}
				}

				data_t z_pred[MEAS_DIM]={0};
				ZPRED_LOOP_I: for(int i=0;i<m;i++)
					ZPRED_LOOP_K: for(int k=0;k<L;k++)
					{
#pragma HLS PIPELINE II = 1
						z_pred[i]+=Wm[k]*Zsigma[i][k];
					}

				// Innovation covariance and cross covariance
				data_t S_mat[MEAS_DIM][MEAS_DIM]={0};
				data_t Pxz[STATE_DIM][MEAS_DIM]={0};
				S_COV_LOOP_I: for(int i=0;i<m;i++)
					S_COV_LOOP_J: for(int j=0;j<m;j++)
						S_COV_LOOP_K: for(int k=0;k<L;k++)
						{
#pragma HLS PIPELINE II = 3
							data_t dz_i = Zsigma[i][k]-z_pred[i];
							data_t dz_j = Zsigma[j][k]-z_pred[j];
							S_mat[i][j]+=Wc[k]*dz_i*dz_j;
						}

					PXZ_LOOP_I: for(int i=0;i<n;i++)
						PXZ_LOOP_J: for(int j=0;j<m;j++)
							PXZ_LOOP_K: for(int k=0;k<L;k++)
							{
#pragma HLS PIPELINE II = 1
								data_t dx = Xsigma_pred[i][k]-x_pred[i];
								data_t dz = Zsigma[j][k]-z_pred[j];
								Pxz[i][j]+=Wc[k]*dx*dz;
							}

						ADD_R_LOOP_I: for(int i=0;i<m;i++) S_mat[i][i]+=R[i][i];

						// Kalman gain (diagonal approx)
						data_t K[STATE_DIM][MEAS_DIM]={0};
						K_LOOP_I: for(int i=0;i<n;i++)
							K_LOOP_J: for(int j=0;j<m;j++)
							{
#pragma HLS PIPELINE II = 1
								K[i][j]=Pxz[i][j]/(S_mat[j][j]+1e-6f);
							}

						// update
						data_t y[MEAS_DIM]={0};
						Y_LOOP_I: for(int i=0;i<m;i++)
							{
#pragma HLS PIPELINE II = 1
								y[i]=z_in[i][t]-z_pred[i];
							}

						UPDATE_LOOP_I: for(int i=0;i<n;i++)
						{
							data_t sum=0;
							UPDATE_LOOP_J: for(int j=0;j<m;j++)
								{
#pragma HLS PIPELINE II = 1
									sum+=K[i][j]*y[j];
								}
							x[i]=x_pred[i]+sum;
						}
                        //

						data_t ks[STATE_DIM][MEAS_DIM];
						#pragma HLS ARRAY_PARTITION variable=ks complete

						// ks = K * S_mat
						for (int i = 0; i < STATE_DIM; i++)
						{
						    for (int j = 0; j < MEAS_DIM; j++)
						    {
						        data_t sum = 0;
						        for (int k = 0; k < MEAS_DIM; k++)
						        {
#pragma HLS PIPELINE II = 1
						            sum += K[i][k] * S_mat[k][j];
						        }
						        ks[i][j] = sum;
						    }
						}

						data_t ksk[6][6];
						#pragma HLS ARRAY_PARTITION variable=ksk complete

						// P_update = ks * K'
						for (int i = 0; i < STATE_DIM; i++)
						{
						    for (int j = 0; j < STATE_DIM; j++)
						    {
						        data_t sum = 0;
						        for (int k = 0; k < MEAS_DIM; k++)
						        {
#pragma HLS PIPELINE II = 1
						            sum += ks[i][k] * K[j][k];
						        }
						        ksk[i][j] = sum;
						    }
						}

						// covariance update (diagonal approx)
						COV_UPDATE_LOOP_I1 : for(int i=0;i<n;i++)
							COV_UPDATE_LOOP_I2 : for(int j=0;j<n;j++)
							{
#pragma HLS PIPELINE II = 1
								P[i][j]=P_pred[i][j] - ksk[i][j];
							}
		}

		// save estimate
		SAVE_EST_LOOP: for(int i=0;i<n;i++) x_est[i][t]=x[i];
	}
}
