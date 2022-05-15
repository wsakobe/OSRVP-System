#ifndef INCLUDE_POSE_ESTIMATION_H_
#define INCLUDE_POSE_ESTIMATION_H_
#define _HAS_STD_BYTE 0
#pragma warning(disable:4996) 

#include "identify_marker.h"

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include "ceres/ceres.h"
#include "glog/logging.h"

#include <fstream>

using namespace ceres;

#define maxLostFrame 3

struct PoseInformation {
	cv::Mat rotation, translation;
	vector<cv::Point3f> tracking_points;
	bool recovery = false;
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

class PoseEstimation {
public:
	PoseEstimation();
	~PoseEstimation();

	PoseInformation poseEstimation(vector<vector<corner_pos_with_ID>> corner_set, vector<CameraParams> camera_parameters, float(*model_3D)[3], unsigned int camera_num);
	void poseEstimationStereo(vector<vector<corner_pos_with_ID>> corner_set, vector<CameraParams> camera_parameters, float(*model_3D)[3]);
	void poseEstimationMono(vector<vector<corner_pos_with_ID>> corner_set, vector<CameraParams> camera_parameters, float(*model_3D)[3]);
	void bundleAdjustment(vector<vector<corner_pos_with_ID>> corner_set, vector<CameraParams> camera_parameters, float(*model_3D)[3]);

	PoseInformation Pose6D;
	cv::Mat end_effector = cv::Mat::zeros(3, 1, CV_32FC1);
	
private:
	void triangulation(const std::vector<cv::Point2f>& points_left, const std::vector<cv::Point2f>& points_right, std::vector<cv::Point3f>& points, CameraParams camera_parameter1, CameraParams camera_parameter2);
	cv::Point2f pixel2cam(const cv::Point2f& p, const cv::Mat& K);
	void buildProblem(Problem* problem, vector<vector<corner_pos_with_ID>> corner_set, vector<CameraParams> camera_parameters, float(*model_3D)[3]);

	int enough_number[5] = { 0 };
	int number_enough_corners = 0;
	cv::Mat rvec, tvec, R;
	vector<cv::Point3f> world_points, pts1, pts2;
	vector<cv::Point2f> image_points, imgpts1, imgpts2;
	int registrated_point_cnt = 0;

	cv::Mat pts_4d;
	vector<cv::Point2f> pts_1, pts_2;

	Problem problem;
	double rot[3], trans[3];
	cv::Mat R, point_cam;
	cv::Mat rvec = cv::Mat::zeros(3, 1, CV_32FC1);
	cv::Mat tvec = cv::Mat::zeros(3, 1, CV_32FC1);
	cv::Mat point_world = cv::Mat::zeros(3, 1, CV_32FC1);
};

class BundleAdjustment {
public:

private:

};

#pragma warning(disable:4996)
#endif