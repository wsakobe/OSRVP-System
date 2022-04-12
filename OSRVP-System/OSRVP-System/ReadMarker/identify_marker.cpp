#include "../include/identify_marker.h"

using namespace std;
using namespace cv;

void IdentifyMarker::identifyMarker(Mat& img, int *p, int(*model_3D)[3])
{
	for (int i = 0; i < 5; i++)
		for (int j = 0; j < 2 * 10; j++)
			for (int k = 0; k < 2 * 10; k++)
				matrix_with_ID[i][j][k] = *(p + i * 2 * 10 * 2 * 10 + j * 2 * 10 + k);

}
