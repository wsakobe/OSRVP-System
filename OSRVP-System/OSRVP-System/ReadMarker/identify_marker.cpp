#include "../include/identify_marker.h"

using namespace std;
using namespace cv;

vector<corner_pos_with_ID> IdentifyMarker::identifyMarker(Mat& img, int *p, vector<cornerInformation> cornerPoints, struct valueMatrix *vm, int (*d)[30])
{
	Mat imgMark(img.rows, img.cols, CV_32FC3);
	cvtColor(img, imgMark, COLOR_GRAY2RGB);

	memset(dot_recovery, -1, sizeof(dot_recovery));
	corner_pos_ID.clear();
	init(p, vm, d);
	for (int i = 0; i < 5; i++) {
		for (int j = 0; j < 2 * number_of_corner_x; j++)
			for (int k = 0; k < 2 * number_of_corner_y; k++) {
				if (matrix_with_ID[i][j][k] != -1) {
					if (checkLattice(i, j, k)) {
						Point2f center_location = (cornerPoints[matrix_with_ID[i][j][k]].point_in_subpixel + cornerPoints[matrix_with_ID[i][j + 1][k]].point_in_subpixel + cornerPoints[matrix_with_ID[i][j][k + 1]].point_in_subpixel + cornerPoints[matrix_with_ID[i][j + 1][k + 1]].point_in_subpixel) / 4;
						Point2f left_location   = (cornerPoints[matrix_with_ID[i][j][k]].point_in_subpixel * 0.85 + cornerPoints[matrix_with_ID[i][j + 1][k]].point_in_subpixel * 0.15 + cornerPoints[matrix_with_ID[i][j][k + 1]].point_in_subpixel * 0.85 + cornerPoints[matrix_with_ID[i][j + 1][k + 1]].point_in_subpixel * 0.15) / 2;
						Point2f up_location     = (cornerPoints[matrix_with_ID[i][j][k]].point_in_subpixel * 0.85 + cornerPoints[matrix_with_ID[i][j + 1][k]].point_in_subpixel * 0.85 + cornerPoints[matrix_with_ID[i][j][k + 1]].point_in_subpixel * 0.15 + cornerPoints[matrix_with_ID[i][j + 1][k + 1]].point_in_subpixel * 0.15) / 2;
						Point2f right_location  = (cornerPoints[matrix_with_ID[i][j][k]].point_in_subpixel * 0.15 + cornerPoints[matrix_with_ID[i][j + 1][k]].point_in_subpixel * 0.85 + cornerPoints[matrix_with_ID[i][j][k + 1]].point_in_subpixel * 0.15 + cornerPoints[matrix_with_ID[i][j + 1][k + 1]].point_in_subpixel * 0.85) / 2;
						Point2f down_location   = (cornerPoints[matrix_with_ID[i][j][k]].point_in_subpixel * 0.15 + cornerPoints[matrix_with_ID[i][j + 1][k]].point_in_subpixel * 0.15 + cornerPoints[matrix_with_ID[i][j][k + 1]].point_in_subpixel * 0.85 + cornerPoints[matrix_with_ID[i][j + 1][k + 1]].point_in_subpixel * 0.85) / 2;
						
						//circle(imgMark, center_location, 1, Scalar(0, 0, 255), -1);
						//circle(imgMark, left_location, 1, Scalar(0, 255, 255), -1);
						//circle(imgMark, up_location, 1, Scalar(0, 255, 255), -1);
						//circle(imgMark, right_location, 1, Scalar(0, 255, 255), -1);
						//circle(imgMark, down_location, 1, Scalar(0, 255, 255), -1);

						pixel_center = img.ptr<float>((int)center_location.y)[(int)center_location.x];
						ave_pixel_around = (img.ptr<float>((int)left_location.y)[(int)left_location.x] + img.ptr<float>((int)right_location.y)[(int)right_location.x] + img.ptr<float>((int)up_location.y)[(int)up_location.x] + img.ptr<float>((int)down_location.y)[(int)down_location.x]) /4;
						
						if (abs(pixel_center - ave_pixel_around) > T_pixel) 
							dot_recovery[i][j][k] = 1;
						else 
							dot_recovery[i][j][k] = 0;
					}
				}
			}
	}	
	
	corner_pos_ID = identifyMarkerPosRANSAC(cornerPoints, 0.85);

	
	for (int i = 0; i < corner_pos_ID.size(); i++) {
		circle(imgMark, corner_pos_ID[i].subpixel_pos, 3, Scalar(255, 0, 0), -1);
		std::stringstream ss;
		ss << corner_pos_ID[i].ID;
		string s = ss.str();
		putText(imgMark, s, corner_pos_ID[i].subpixel_pos + Point2f(2, 2), FONT_HERSHEY_SIMPLEX, 0.3, Scalar(0, 255, 0), 1.8);
	}
	imshow("Identify", imgMark);
	waitKey(1);
	
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
			if (dot_recovery[label][x + j][y + i]) value += key;
			key <<= 1;
		}
	if ((x + y) % 2) value += key;
	return value;
}

float IdentifyMarker::recoveryMatrixRatio(int label, int x, int y, int value)
{
	number_all  = 0;
	number_succ = 0;
	for (int j = 0; j < 2 * number_of_corner_x; j++)
		for (int k = 0; k < 2 * number_of_corner_y; k++)
			if (dot_recovery[label][j][k] != -1) {
				number_all++;
				if (dot_recovery[label][j][k] == dot_matrix[value_matrix[value].pos.x + dir[value_matrix[value].dir][0] * (j - x) + dir[value_matrix[value].dir][1] * (k - y) + bias[value_matrix[value].dir][0]][value_matrix[value].pos.y + dir[value_matrix[value].dir][2] * (j - x) + dir[value_matrix[value].dir][3] * (k - y) + bias[value_matrix[value].dir][1]])
					number_succ++;
			}
	return (1.0 * number_succ) / (1.0 * number_all);
}

vector<corner_pos_with_ID> IdentifyMarker::identifyMarkerPosRANSAC(vector<cornerInformation> cornerPoints, float threshold)
{
	for (int i = 0; i < 5; i++) {
		for (int j = 0; j < 2 * number_of_corner_x; j++)
			for (int k = 0; k < 2 * number_of_corner_y; k++) {
				if (dot_recovery[i][j][k] != -1) {
					if (checkGrid3(i, j, k)) {
						matrix_value = extractMatrixValue(i, j, k);
						if (recoveryMatrixRatio(i, j, k, matrix_value) > threshold) {
							countCornerPosWithID(i, j, k, matrix_value, cornerPoints);
							j = 2 * number_of_corner_x;
							k = 2 * number_of_corner_y; //break two fors
						}
					}
				}
			}
	}
	return corner_pos_ID;
}

void IdentifyMarker::countCornerPosWithID(int label, int x, int y, int value, vector<cornerInformation> cornerPoints)
{
	extern int number_of_corner_x_input, number_of_corner_y_input;
	corner_pos_with_ID corner_temp;
	for (int j = 0; j < 2 * number_of_corner_x; j++)
		for (int k = 0; k < 2 * number_of_corner_y; k++) {
			if (matrix_with_ID[label][j][k] != -1) {
				corner_temp.label = label;
				corner_temp.ID = (value_matrix[value].pos.x + dir[value_matrix[value].dir][0] * (j - x) + dir[value_matrix[value].dir][1] * (k - y)) * (number_of_corner_y_input + 1) + value_matrix[value].pos.y + dir[value_matrix[value].dir][2] * (j - x) + dir[value_matrix[value].dir][3] * (k - y) + 1;
				corner_temp.subpixel_pos = cornerPoints[matrix_with_ID[label][j][k]].point_in_subpixel;
				corner_pos_ID.push_back(corner_temp);
			}			
		}
}

void IdentifyMarker::init(int* p, struct valueMatrix* vm, int(*d)[30])
{
	memset(dot_recovery, -1, sizeof(dot_recovery));

	for (int i = 0; i < 5; i++)
		for (int j = 0; j < 2 * number_of_corner_x; j++)
			for (int k = 0; k < 2 * number_of_corner_y; k++)
				matrix_with_ID[i][j][k] = *(p + i * 2 * number_of_corner_x * 2 * number_of_corner_x + j * 2 * number_of_corner_y + k);
	
	for (int i = 0; i < 2 * number_of_corner_x; i++)
		for (int j = 0; j < 2 * number_of_corner_y; j++)
			dot_matrix[i][j] = d[i][j];
	
	for (int i = 0; i < 1025; i++)
		value_matrix[i] = vm[i];
}
