#pragma once
#ifndef INCLUDE_POSE_ESTIMATION_H_
#define INCLUDE_POSE_ESTIMATION_H_
#define _HAS_STD_BYTE 0
#pragma warning(disable:4996) 

#include "config.h"

using namespace ceres;
using namespace std;

class PoseEstimation {
public:
	PoseEstimation();
	~PoseEstimation();

	FinalPoseInformation poseEstimation(vector<cornerMarkerInfo> corners_all, vector<CameraParams> camera_parameters, vector<ModelInfo> model, unsigned int camera_num, bool camera_type);
	void poseEstimationStereo(vector<cornerMarkerInfo> corner_set, vector<CameraParams> camera_parameters, ModelInfo model, int pose_type); // 0 - robot; 1 - opener; 2 - endo;
	void poseEstimationMono(vector<cornerMarkerInfo> corner_set, vector<CameraParams> camera_parameters, ModelInfo model, int pose_type);
	void poseEstimationPlanar(cornerMarkerInfo corner_set, vector<CameraParams> camera_parameters);
	void bundleAdjustment(vector<cornerMarkerInfo> corner_set, vector<CameraParams> camera_parameters, ModelInfo model, int pose_type);

	PoseInformation Pose6D;
	FinalPoseInformation Pose_final;
	cv::Mat end_effector = cv::Mat::zeros(3, 1, CV_32FC1);
	
private:
	void triangulation(const std::vector<cv::Point2f>& points_left, const std::vector<cv::Point2f>& points_right, std::vector<cv::Point3f>& points, CameraParams camera_parameter1, CameraParams camera_parameter2);
	cv::Point2f pixel2cam(const cv::Point2f& p, const cv::Mat& K);
	void buildProblem(Problem* problem, vector<vector<corner_pos_with_ID>> corner_set, vector<CameraParams> camera_parameters, ModelInfo model);
	
	int enough_number[5] = { 0 };
	int number_enough_corners = 0;
	cv::Mat rvec, tvec, R, R_cam, Rotcam;
	vector<cv::Point3f> world_points, pts1, pts2;
	vector<cv::Point2f> image_points, imgpts1, imgpts2;
	vector<cv::Point2f> corners_ori, corners_undistort;
	vector<vector<corner_pos_with_ID>> corner_set;
	int registrated_point_cnt = 0;

	cv::Mat pts_4d;
	vector<cv::Point2f> pts_1, pts_2;

	Problem problem;
	double rot[3], trans[3], modelpoint[3];
};

#pragma warning(disable:4996)
#endif