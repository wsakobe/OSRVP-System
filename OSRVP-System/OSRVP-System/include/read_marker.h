#pragma once
#ifndef INCLUDE_READ_MARKER_H_
#define INCLUDE_READ_MARKER_H_

#include "config.h"
#include "pre_filter.h"
#include "final_election.h"
#include "corner_array_organization.h"
#include "identify_marker.h"

using namespace cv;
using namespace std;

class ReadMarker {
public:
	cornerMarkerInfo readMarker(Mat& image, bool cameraType, DynamicROIBox box);
	void dynamicROI(FinalPoseInformation Pose, vector<DynamicROIBox>& Box, CameraParams camera_parameters, vector<ModelInfo> model_3D, bool cameraType);

private:
	vector<Point3f> axesPoints;
	vector<Point2f> imagePoints;
	Mat Rot, rvec, image_gray;
	int BoxBorder = 30;
	int cnt = 1, x_min = 10000, y_min = 10000, x_max = -1, y_max = -1;

	int number_of_corner_x_input, number_of_corner_y_input;
	imageParams ImgParams, ImgParamsOri, ImgParamsEndo;
	valueMatrix value_matrix[1025];
	int dot_matrix[2 * number_of_corner_x][2 * number_of_corner_y];
	vector<cornerPreInfo> candidate_corners;
	vector<cornerInformation> cornerPoints;
	vector<corner_pos_with_ID> corner_pos;
};

#endif