#ifndef INCLUDE_POSE_ESTIMATION_H_
#define INCLUDE_POSE_ESTIMATION_H_

#include "identify_marker.h"

struct PoseInformation {
	Mat rotation, translation;
	vector<Point3f> tracking_points;
};

struct DynamicROIBox {
	Point position;
	int width, height;
	int lostFrame = 0;
};

class PoseEstimation {
public:
	PoseInformation poseEstimationStereo(vector<corner_pos_with_ID> corner_set_left, vector<corner_pos_with_ID> corner_set_right, Mat IntrinsicCamLeft, Mat DistcoeffLeft, Mat IntrinsicCamRight, Mat DistcoeffRight, Mat Rot, Mat Trans, float(*model_3D)[3]);
	PoseInformation poseEstimationMono(vector<corner_pos_with_ID> corner_set, Mat IntrinsicCam, Mat DistCoeff, float(*model_3D)[3]);

	PoseInformation Pose6D;


private:
	void triangulation(const std::vector<Point2f>& points_left, const std::vector<Point2f>& points_right, std::vector<Point3f>& points, Mat IntrinsicCamLeft, Mat IntrinsicCamRight, Mat Rot, Mat Trans);
	Point2f pixel2cam(const Point2f& p, const Mat& K);

	Mat rvec, tvec, R;
	vector<Point3f> world_points, pts1, pts2;
	vector<Point2f> image_points, imgpts1, imgpts2;
	int registrated_point_cnt;

	Mat pts_4d;
	vector<Point2f> pts_1, pts_2;
};

#pragma warning(disable:4996)
#endif