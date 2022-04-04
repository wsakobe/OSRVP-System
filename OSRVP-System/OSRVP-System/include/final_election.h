#ifndef INCLUDE_FINAL_ELECTION_H_
#define INCLUDE_FINAL_ELECTION_H_

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <algorithm>
#include <cstdlib>
#include <vector>
#include <iostream> 

using namespace cv;
using namespace std;

struct cornerInformation {
	Point   point_in_pixel;
	Point2f point_in_subpixel;
	float angle_black_edge = -1.0;
	float angle_white_edge = -1.0;
};

class FinalElection {
public:
	FinalElection();
	~FinalElection();
	
	vector<cornerInformation> cornerPoints;

	vector<cornerInformation> finalElection(Mat& img, vector<Point> candidate_corners);
	void subpixelRefinement(Mat& img, vector<Point> candidate_corners);
	void fitQuadraticSurface(Mat& img);
	void templateMatching(Mat& img);

private:
	Mat tmp, img_neighbor, grad_neighbor, grad_neighbor_x, grad_neighbor_y, surface_temp_x, surface_temp_y, surface_temp;
	Mat B, subpixel;
	int maskTem = 11;
	int maskR = 5;
	int maskSurface = 7;
	int maskSurfaceL = 2 * maskSurface + 1;
	Mat grad_row, surface_row;
	double result;
	cornerInformation cur;
};

#endif