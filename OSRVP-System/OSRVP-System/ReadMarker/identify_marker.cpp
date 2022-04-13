#include "../include/identify_marker.h"

using namespace std;
using namespace cv;

vector<corner_pos_with_ID> IdentifyMarker::identifyMarker(Mat& img, int *p, vector<cornerInformation> cornerPoints, struct valueMatrix *value_matrix, int (*dot_matrix)[10])
{
	memset(dot_recovery, -1, sizeof(dot_recovery));

	for (int i = 0; i < 5; i++)
		for (int j = 0; j < 2 * 10; j++)
			for (int k = 0; k < 2 * 10; k++)
				matrix_with_ID[i][j][k] = *(p + i * 2 * 10 * 2 * 10 + j * 2 * 10 + k);

	for (int i = 0; i < 5; i++) {
		isEnd = true;
		for (int j = 0; j < 2 * 10; j++)
			for (int k = 0; k < 2 * 10; k++) {
				if (matrix_with_ID[i][j][k] != -1) {
					isEnd = false;
					if (checkLattice(i, j, k)) {
						Point2f center_location = (cornerPoints[matrix_with_ID[i][j][k]].point_in_subpixel + cornerPoints[matrix_with_ID[i][j + 1][k]].point_in_subpixel + cornerPoints[matrix_with_ID[i][j][k + 1]].point_in_subpixel + cornerPoints[matrix_with_ID[i][j + 1][k + 1]].point_in_subpixel) / 4;
						Point2f left_location   = (cornerPoints[matrix_with_ID[i][j][k]].point_in_subpixel * 0.8 + cornerPoints[matrix_with_ID[i][j + 1][k]].point_in_subpixel * 0.2 + cornerPoints[matrix_with_ID[i][j][k + 1]].point_in_subpixel * 0.8 + cornerPoints[matrix_with_ID[i][j + 1][k + 1]].point_in_subpixel * 0.2) / 2;
						Point2f up_location     = (cornerPoints[matrix_with_ID[i][j][k]].point_in_subpixel * 0.8 + cornerPoints[matrix_with_ID[i][j + 1][k]].point_in_subpixel * 0.8 + cornerPoints[matrix_with_ID[i][j][k + 1]].point_in_subpixel * 0.2 + cornerPoints[matrix_with_ID[i][j + 1][k + 1]].point_in_subpixel * 0.2) / 2;
						Point2f right_location  = (cornerPoints[matrix_with_ID[i][j][k]].point_in_subpixel * 0.2 + cornerPoints[matrix_with_ID[i][j + 1][k]].point_in_subpixel * 0.8 + cornerPoints[matrix_with_ID[i][j][k + 1]].point_in_subpixel * 0.2 + cornerPoints[matrix_with_ID[i][j + 1][k + 1]].point_in_subpixel * 0.8) / 2;
						Point2f down_location   = (cornerPoints[matrix_with_ID[i][j][k]].point_in_subpixel * 0.2 + cornerPoints[matrix_with_ID[i][j + 1][k]].point_in_subpixel * 0.2 + cornerPoints[matrix_with_ID[i][j][k + 1]].point_in_subpixel * 0.8 + cornerPoints[matrix_with_ID[i][j + 1][k + 1]].point_in_subpixel * 0.8) / 2;
						
						pixel_center = img.ptr<float>((int)center_location.y)[(int)center_location.x];
						ave_pixel_around = (img.ptr<float>((int)left_location.y)[(int)left_location.x] + img.ptr<float>((int)right_location.y)[(int)right_location.x] + img.ptr<float>((int)up_location.y)[(int)up_location.x] + img.ptr<float>((int)down_location.y)[(int)down_location.x]) /4;
						
						if (abs(pixel_center - ave_pixel_around) > T_pixel) 
							dot_recovery[i][j][k] = 1;
						else 
							dot_recovery[i][j][k] = 0;
					}
				}
			}
		if (isEnd) break;
	}	
	
	corner_pos_ID = identifyMarkerPosRANSAC();
	return corner_pos_ID;
}

inline bool IdentifyMarker::checkLattice(int label, int x, int y)
{
	if ((matrix_with_ID[label][x + 1][y] != -1) && (matrix_with_ID[label][x][y + 1] != -1) && (matrix_with_ID[label][x + 1][y + 1] != -1)) return true;
	else return false;
}

inline bool IdentifyMarker::checkGrid3(int label, int x, int y)
{
	for (int i = x; i < x + 3; i++)
		for (int j = y; j < y + 3; j++)
			if (dot_recovery[label][i][j] == -1) return false;
	return true;
}

int IdentifyMarker::extractMatrixValue(int label, int x, int y)
{

}

float IdentifyMarker::recoveryMatrixRatio(int label, int value)
{

}

vector<corner_pos_with_ID> IdentifyMarker::identifyMarkerPosRANSAC()
{
	for (int i = 0; i < 5; i++) {
		isEnd = true;
		for (int j = 0; j < 2 * 10; j++)
			for (int k = 0; k < 2 * 10; k++) {
				if (dot_recovery[i][j][k] != -1) {
					isEnd = false;
					if (checkGrid3(i, j, k)) {
						matrix_value = extractMatrixValue(i, j, k);
						if (recoveryMatrixRatio(i, matrix_value) > 0.8) {
							countCornerPosWithID(i);
							break;
						}
					}
				}
			}
		if (isEnd) break;
	}
	return corner_pos_ID;
}

void IdentifyMarker::countCornerPosWithID(int label)
{
	for (int j = 0; j < 2 * 10; j++)
		for (int k = 0; k < 2 * 10; k++) {
			if (matrix_with_ID)
		}
}
