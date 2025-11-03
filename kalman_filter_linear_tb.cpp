#include "kalman_filter_linear.h"

using namespace std;
//
int main()
{
	static data_t z_res[3][TMAX];
	static data_t x_res[state_dim][TMAX];
#pragma HLS ARRAY_PARTITION variable=z_res complete
#pragma HLS ARRAY_PARTITION variable=x_res complete

	cout << "-----------------------------------------------" << endl;
	//
	FILE *fp_z = fopen("z_matrix.bin", "rb");
	if (fp_z == NULL)printf("z_matrix.bin!\n");
	for (int idx1 = 0; idx1 < 3; idx1++)
	{
		for (int idx2 = 0; idx2 < TMAX; idx2++)
		{
#pragma HLS PIPELINE II= 5
			fread(&z_res[idx1][idx2], sizeof(float), 1, fp_z);
		}
	}
	fclose(fp_z);
	//
	FILE *fp_x = fopen("x_matrix.bin", "rb");
	if (fp_x == NULL)printf("x_matrix.bin!\n");
	for (int idx1 = 0; idx1 < state_dim; idx1++)
	{
		for (int idx2 = 0; idx2 < TMAX; idx2++)
		{
#pragma HLS PIPELINE II= 5
			fread(&x_res[idx1][idx2], sizeof(float), 1, fp_x);
		}
	}
	fclose(fp_x);
	//
	static data_t z[3];
	data_t x_est[6];
	bool read;
	bool write;
	static data_t sat_out[1];
	static int cnt_read = 0;
	static int cnt_write = 0;
	//

	while(sat_out[0] < 4)
	{
		if (read == 1)
		{
			z[0] = z_res[0][cnt_read];
			z[1] = z_res[1][cnt_read];
			z[2] = z_res[2][cnt_read];
			cnt_read++;
		}

		kalman_filter_linier(z,x_est,read,write,sat_out);

		if(write == 1)
		{
			for (int idx1 = 0; idx1 < 6; idx1++)
			{
				data_t er = abs(x_est[idx1]-x_res[idx1][cnt_write]);
				if(er > 1e-6)
				{
					cout << " cnt_write = "  << cnt_write << " ,  "  << x_est[idx1] << " ,  "  << x_res[idx1][cnt_write];
					cout << "er = " << er << endl;
				}
			}
			cnt_write++;
		}
	}

	cout << "-----------------------------------------------" << endl;
	return 0;
}
