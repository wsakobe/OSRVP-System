#ifndef INCLUDE_IDENTIFY_MARKER_H_
#define INCLUDE_IDENTIFY_MARKER_H_

#include "corner_array_organization.h"

struct valueMatrix {
	Point pos;
	int dir;
};

struct corner_pos_with_ID {
	Point2f subpixel_pos;
	int label;
	int ID;
};

class OrganizationArray;

class IdentifyMarker {
public:
	vector<corner_pos_with_ID> identifyMarker(Mat& img, int *p, vector<cornerInformation> cornerPoints, struct valueMatrix *value_matrix, int (*dot_matrix)[10]);
	vector<corner_pos_with_ID> corner_pos_ID;

private:
	bool checkLattice(int label, int x, int y);
	bool checkGrid3(int label, int x, int y);
	int  extractMatrixValue(int label, int x, int y);
	float recoveryMatrixRatio(int label, int value);
	vector<corner_pos_with_ID> identifyMarkerPosRANSAC();
	void countCornerPosWithID(int label);

	bool isEnd;
	float pixel_center, ave_pixel_around, T_pixel = 0.25;
	int matrix_with_ID[5][2 * 10][2 * 10], dot_recovery[5][2 * 10][2 * 10];
	int matrix_value;
};

#endif