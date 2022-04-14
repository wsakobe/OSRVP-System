#include "../include/identify_marker.h"

using namespace std;
using namespace cv;

vector<corner_pos_with_ID> IdentifyMarker::identifyMarker(Mat& img, int *p, vector<cornerInformation> cornerPoints, struct valueMatrix *vm, int (*d)[10])
{
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
	
	corner_pos_ID = identifyMarkerPosRANSAC(cornerPoints);
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
	int value = 0, key = 1;
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++) {
			if (dot_recovery[label][x + i][y + j]) value += key;
			key << 1;
		}
	if ((x + y) % 2) value += key;
	return value;
}

float IdentifyMarker::recoveryMatrixRatio(int label, int x, int y, int value)
{
	number_all  = 0;
	number_succ = 0;
	for (int j = 0; j < 2 * 10; j++)
		for (int k = 0; k < 2 * 10; k++)
			if (dot_recovery[label][j][k] != -1) {
				number_all++;
				if (dot_recovery[label][j][k] == dot_matrix[value_matrix[value].pos.x + bias[value_matrix[value].dir][0] * (j - x) + bias[value_matrix[value].dir][1] * (k - y)][value_matrix[value].pos.y + bias[value_matrix[value].dir][2] * (j - x) + bias[value_matrix[value].dir][3] * (k - y)]) 
					number_succ++;
			}
	return number_succ / number_all;
}

vector<corner_pos_with_ID> IdentifyMarker::identifyMarkerPosRANSAC(vector<cornerInformation> cornerPoints)
{
	for (int i = 0; i < 5; i++) {
		isEnd = true;
		for (int j = 0; j < 2 * 10; j++)
			for (int k = 0; k < 2 * 10; k++) {
				if (dot_recovery[i][j][k] != -1) {
					isEnd = false;
					if (checkGrid3(i, j, k)) {
						matrix_value = extractMatrixValue(i, j, k);
						if (recoveryMatrixRatio(i, j, k, matrix_value) > 0.9) {
							countCornerPosWithID(i, j, k, matrix_value, cornerPoints);
							break;
						}
					}
				}
			}
		if (isEnd) break;
	}
	return corner_pos_ID;
}

void IdentifyMarker::countCornerPosWithID(int label, int x, int y, int value, vector<cornerInformation> cornerPoints)
{
	corner_pos_with_ID corner_temp;
	for (int j = 0; j < 2 * 10; j++)
		for (int k = 0; k < 2 * 10; k++) {
			if (matrix_with_ID[label][j][k] != -1) {
				corner_temp.label = label;
				corner_temp.ID = (value_matrix[value].pos.x + bias[value_matrix[value].dir][0] * (j - x) + bias[value_matrix[value].dir][1] * (k - y)) * 10 + value_matrix[value].pos.y + bias[value_matrix[value].dir][2] * (j - x) + bias[value_matrix[value].dir][3] * (k - y);
				corner_temp.subpixel_pos = cornerPoints[matrix_with_ID[label][j][k]].point_in_subpixel;
				corner_pos_ID.push_back(corner_temp);
			}			
		}
}

void IdentifyMarker::init(int* p, struct valueMatrix* vm, int(*d)[10])
{
	memset(dot_recovery, -1, sizeof(dot_recovery));

	for (int i = 0; i < 5; i++)
		for (int j = 0; j < 2 * 10; j++)
			for (int k = 0; k < 2 * 10; k++)
				matrix_with_ID[i][j][k] = *(p + i * 2 * 10 * 2 * 10 + j * 2 * 10 + k);
	
	for (int i = 0; i < 2 * 10; i++)
		for (int j = 0; j < 2 * 10; j++)
			dot_matrix[i][j] = d[i][j];
	
	for (int i = 0; i < 1025; i++)
		value_matrix[i] = vm[i];
}
