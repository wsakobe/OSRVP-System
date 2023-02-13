#pragma once
#ifndef INCLUDE_CONFIG_H_
#define INCLUDE_CONFIG_H_

// header
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <algorithm>
#include <cstdlib>
#include <vector>
#include <iostream> 
#include <fstream>
#include <algorithm>
#include <process.h>
#include <conio.h>
#include <thread>

#include "mvcameracontrol.h"

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include "ceres/ceres.h"
#include "glog/logging.h"

using namespace ceres;
using namespace std;

// define
#define maxLostFrame 3
#define HikingCamera 0
#define USBCamera 1
#define number_of_corner_x 15
#define number_of_corner_y 15

#define PI 3.1415926535

// class
struct corner_pos_with_ID {
	cv::Point2f subpixel_pos;
	int label;
	int ID;
};

struct PoseInformation {
	cv::Mat rotation, translation;
	vector<cv::Point3f> tracking_points;
	bool recovery = false;
};

struct FinalPoseInformation {
	PoseInformation robot_pose;
	PoseInformation opener_pose;
	PoseInformation endo_pose;
};

struct ModelInfo {
	int MarkerID; //0 - suture robot; 1 - plant robot; 2 - opener; 3 - endoscope
	vector<cv::Point3f> corners;
};

struct DynamicROIBox {
	cv::Point position;
	int width, height;
	int lostFrame = 0;
};

struct CameraParams {
	cv::Mat Intrinsic = cv::Mat::zeros(3, 3, CV_32FC1);
	cv::Mat Distortion = cv::Mat::zeros(5, 1, CV_32FC1);
	cv::Mat Rotation = cv::Mat::eye(3, 3, CV_32FC1);
	cv::Mat Translation = cv::Mat::zeros(3, 1, CV_32FC1);
};

struct valueMatrix {
	cv::Point pos;
	int dir;
};

struct imageParams {	// Basic Image Parameters
	int width, height;
};

struct cornerMarkerInfo {
	vector<corner_pos_with_ID> robot_marker;
	vector<corner_pos_with_ID> opener_marker;
	vector<corner_pos_with_ID> endoscope_marker;
};

struct cornerInformation {
	cv::Point   point_in_pixel;
	cv::Point2f point_in_subpixel;
	float angle_black_edge = -1.0;
	float angle_white_edge = -1.0;
	float hessian_response_score, template_response_score;
};

struct cornerPreInfo {
	cv::Point corner_position;
	float response_score;
};

// variables
extern int number_of_corner_x_input, number_of_corner_y_input;

extern cornerMarkerInfo corners;
extern imageParams ImgParams, ImgParamsOri, ImgParamsEndo;
extern vector<cornerPreInfo> candidate_corners;
extern vector<cornerInformation> cornerPoints;
extern vector<corner_pos_with_ID> corner_pos;
extern vector<vector<corner_pos_with_ID>> corner_pos_ID;

extern FinalPoseInformation final_pose, Poses;

extern valueMatrix value_matrix[1025];
extern int dot_matrix[2 * number_of_corner_x][2 * number_of_corner_y];

extern vector<cornerMarkerInfo> corners_all;

#endif