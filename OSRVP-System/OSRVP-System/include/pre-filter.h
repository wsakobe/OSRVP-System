#ifndef INCLUDE_PRE_FILTER_H_
#define INCLUDE_PRE_FILTER_H_

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

#include <algorithm>
#include <cstdlib>
#include <vector>
#include <iostream> 

#define number_of_corner_x 30
#define number_of_corner_y 30

using namespace std;
using namespace cv;

struct imageParams {	// Basic Image Parameters
	int width, height;
};

struct cornerPreInfo {
	Point corner_position;
	float response_score;
};

class PreFilter {
public:
	int maskR = 6;
	int kernal_size = 7;
	int sigma = 3;
	Mat Gx, Gy, Gxx, Gyy, Gxy, G_score, G_score_after_NMS, score_sequence, img_blur;
	vector<cornerPreInfo> preFilter(Mat& image, int number); // A Pre-Filter based on image gradient and Hessian matrix

private:
	vector<cornerPreInfo> corners;
	cornerPreInfo temporal_corner;
	float G_filtermin, G_filtermax;

	priority_queue <float, vector<float>, less<float> > Q;
};

#endif // INCLUDE_PRE_FILTER_H_