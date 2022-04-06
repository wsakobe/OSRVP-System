#ifndef INCLUDE_PRE_FILTER_H_
#define INCLUDE_PRE_FILTER_H_

#include "final_election.h"

using namespace std;
using namespace cv;

class PreFilter {
public:
	int maskR = 5;
	int kernal_size = 7;
	int sigma = 3;
	Mat Gx, Gy, Gxx, Gyy, Gxy, G_score, G_score_after_NMS, score_sequence, img_blur;
	vector<Point> preFilter(Mat& image, int number); // A Pre-Filter based on image gradient and Hessian matrix

private:
	vector<Point> candidate_corners;
};

#endif // INCLUDE_PRE_FILTER_H_
