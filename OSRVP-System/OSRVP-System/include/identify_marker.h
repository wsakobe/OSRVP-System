#pragma once
#ifndef INCLUDE_IDENTIFY_MARKER_H_
#define INCLUDE_IDENTIFY_MARKER_H_

#include "config.h"

class IdentifyMarker {
public:
	cornerMarkerInfo identifyMarker(cv::Mat& img, int *p, vector<cornerInformation> cornerPoints, struct valueMatrix *value_matrix, int (*dot_matrix)[30], bool cameraType);
	vector<corner_pos_with_ID> corner_pos_ID;
	cornerMarkerInfo corners;

private:
	bool checkLattice(int label, int x, int y);
	bool checkGrid3(int label, int x, int y);
	int  extractMatrixValue(int label, int x, int y);
	float recoveryMatrixRatio(int label, int x, int y, int value);
	cornerMarkerInfo identifyMarkerPosRANSAC(vector<cornerInformation> cornerPoints, float threshold, bool cameraType);
	void countCornerPosWithID(int label, int x, int y, int value, vector<cornerInformation> cornerPoints, bool cameraType);
	void init_data(int* p, struct valueMatrix* vm, int(*d)[30]);

	float pixel_center, ave_pixel_around, T_pixel_far = 0.25, T_pixel_near = 0.1;
	int matrix_with_ID[5][2 * number_of_corner_x][2 * number_of_corner_y], dot_recovery[5][2 * number_of_corner_x][2 * number_of_corner_y];
	int matrix_value, number_all, number_succ;

	int dir[4][4]  = { 1, 0, 0, 1,  0, -1, 1, 0,  -1, 0, 0, -1,  0, 1, -1, 0 };
	int bias[4][2] = { 0,0, -1,0, -1,-1, 0,-1 };

	valueMatrix value_matrix[1025];
	int dot_matrix[2 * number_of_corner_x][2 * number_of_corner_y];
	int number_of_corner_x_input, number_of_corner_y_input;
};

#endif