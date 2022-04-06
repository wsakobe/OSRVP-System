#ifndef INCLUDE_FINAL_ELECTION_H_
#define INCLUDE_FINAL_ELECTION_H_

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <algorithm>
#include <cstdlib>
#include <vector>
#include <iostream> 

#define PI 3.1415926535

using namespace cv;
using namespace std;

struct cornerInformation {
	Point   point_in_pixel;
	Point2f point_in_subpixel;
	float angle_black_edge = -1.0;
	float angle_white_edge = -1.0;
	float response_score;
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
	Mat hypersurface_temp, hypersurface_temp_x2, hypersurface_temp_y2, hypersurface_temp_xy, hypersurface_temp_x, hypersurface_temp_y;
	Mat B, subpixel, hypersurface_coeffs;
	Mat grad_row, surface_row;
	Mat img_hypersurface;
	Mat coeffs, roots;

	int maskTemR = 6, maskTemp = 13;
	int maskR = 5;
	int maskSurface = 7;
	int maskSurfaceL = 2 * maskSurface + 1;
	int edgeIdx, directIdx;
	
	double result;
	float angle1, angle2, edge_angle, direction_angle;
	float response_score_max = -1.0, T_response = 0.3;

	cornerInformation cur;	
};

#endif