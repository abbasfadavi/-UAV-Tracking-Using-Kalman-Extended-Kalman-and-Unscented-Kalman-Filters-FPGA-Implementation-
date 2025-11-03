#include "kalman_filter_extended.h"

using namespace std;
//
int main()
{
	static data_t z_res[3][TMAX];
	static data_t x_res[6][TMAX];
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
	for (int idx1 = 0; idx1 < 6; idx1++)
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
	bool read;
	bool write;
	static data_t sat_out[1];
	static int cnt_read = 0;
	static int cnt_write = 0;
	static data_t x_est[6][TMAX];
#pragma HLS ARRAY_PARTITION variable=x_res complete
	//
	kalman_filter_extended(z_res,x_est);

		for (int i = 0; i < TMAX; i++)
			for (int j = 0; j < 6; j++)
			{
				data_t er = abs(x_res[j][i] - x_est[j][i]);

				if(er > 1e-3)
				{
					//cout <<  er << endl;
					cout << " , " << x_res[j][i] << " , " << x_est[j][i] << " , " << er << endl;
				}
			}

	cout << "-----------------------------------------------" << endl;
	return 0;
}
