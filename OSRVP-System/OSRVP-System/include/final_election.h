#pragma once
#ifndef INCLUDE_FINAL_ELECTION_H_
#define INCLUDE_FINAL_ELECTION_H_

#include "config.h"

using namespace std;

class FinalElection {
public:
	FinalElection();
	~FinalElection();
	
	vector<cornerInformation> cornerPoints;

	vector<cornerInformation> finalElection(cv::Mat& img, vector<cornerPreInfo> candidate_corners);
	void subpixelRefinement(cv::Mat& img);
	void fitQuadraticSurface(cv::Mat& img, vector<cornerPreInfo> candidate_corners);
	void templateMatching(cv::Mat& img);

private:
	cv::Mat tmp, img_neighbor, grad_neighbor, grad_neighbor_x, grad_neighbor_y, surface_temp_x, surface_temp_y, surface_temp;
	cv::Mat hypersurface_temp, hypersurface_temp_x2, hypersurface_temp_y2, hypersurface_temp_xy, hypersurface_temp_x, hypersurface_temp_y;
	cv::Mat B, subpixel, hypersurface_coeffs;
	cv::Mat grad_row, surface_row;
	cv::Mat img_hypersurface;
	cv::Mat coeffs, roots;

	int maskTemR = 6, maskTemp = 13;
	int maskR = 5;
	int maskSurface = 5;
	int maskSurfaceL = 2 * maskSurface + 1;
	int edgeIdx, directIdx;
	
	double result;
	float angle1, angle2, edge_angle, direction_angle;
	float lamda = 1, template_response_score_max = -1.0, hessian_response_score_max = 0, T_response = 0.3, T_temp_max = 0.85;

	cornerInformation cur;	
};

#endif