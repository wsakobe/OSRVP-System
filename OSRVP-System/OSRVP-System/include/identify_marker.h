#ifndef INCLUDE_IDENTIFY_MARKER_H_
#define INCLUDE_IDENTIFY_MARKER_H_

#include "corner_array_organization.h"

class OrganizationArray;

class IdentifyMarker {
public:
	void identifyMarker(Mat& img, int *p, int (*model_3D)[3]);

private:
	int matrix_with_ID[5][2 * 10][2 * 10];
};

#endif