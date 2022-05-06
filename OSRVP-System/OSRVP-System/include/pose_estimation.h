#ifndef INCLUDE_POSE_ESTIMATION_H_
#define INCLUDE_POSE_ESTIMATION_H_

#include "identify_marker.h"
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <fstream>

#define maxLostFrame 3

struct PoseInformation {
	Mat rotation, translation;
	vector<Point3f> tracking_points;
	bool recovery = false;
};

struct DynamicROIBox {
	Point position;
	int width, height;
	int lostFrame = 0;
};

struct CameraParams {
	Mat Intrinsic = Mat::zeros(3, 3, CV_32FC1);
	Mat Distortion = Mat::zeros(5, 1, CV_32FC1);
	Mat Rotation = Mat::eye(3, 3, CV_32FC1);
	Mat Translation = Mat::zeros(3, 1, CV_32FC1);
};

class PoseEstimation {
public:
	PoseInformation poseEstimation(vector<vector<corner_pos_with_ID>> corner_set, vector<CameraParams> camera_parameters, float(*model_3D)[3], unsigned int camera_num);
	void poseEstimationStereo(vector<vector<corner_pos_with_ID>> corner_set, vector<CameraParams> camera_parameters, float(*model_3D)[3], int camera_number[5]);
	void poseEstimationMono(vector<vector<corner_pos_with_ID>> corner_set, vector<CameraParams> camera_parameters, float(*model_3D)[3], int camera_number);
	
	PoseInformation Pose6D;
	Mat end_effector = Mat::zeros(3, 1, CV_32FC1);
	
private:
	void triangulation(const std::vector<Point2f>& points_left, const std::vector<Point2f>& points_right, std::vector<Point3f>& points, CameraParams camera_parameter1, CameraParams camera_parameter2);
	Point2f pixel2cam(const Point2f& p, const Mat& K);

	Mat rvec, tvec, R;
	vector<Point3f> world_points, pts1, pts2;
	vector<Point2f> image_points, imgpts1, imgpts2;
	int registrated_point_cnt = 0;

	Mat pts_4d;
	vector<Point2f> pts_1, pts_2;
};

class BundleAdjustment {
public:

private:

};

#pragma warning(disable:4996)
#endif