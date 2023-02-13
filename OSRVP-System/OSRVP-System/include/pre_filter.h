#pragma once
#ifndef INCLUDE_PRE_FILTER_H_
#define INCLUDE_PRE_FILTER_H_

#include "config.h"

using namespace std;

class PreFilter {
public:
	int maskR = 6;
	int kernal_size = 7;
	int sigma = 3;
	cv::Mat Gx, Gy, Gxx, Gyy, Gxy, G_score, G_score_after_NMS, score_sequence, img_blur;
	vector<cornerPreInfo> preFilter(cv::Mat& image, int number); // A Pre-Filter based on image gradient and Hessian matrix

private:
	vector<cornerPreInfo> corners;
	cornerPreInfo temporal_corner;
	float G_filtermin, G_filtermax;

	priority_queue <float, vector<float>, less<float> > Q;
};

#endif // INCLUDE_PRE_FILTER_H_