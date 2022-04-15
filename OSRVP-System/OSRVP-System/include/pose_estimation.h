#ifndef INCLUDE_POSE_ESTIMATION_H_
#define INCLUDE_POSE_ESTIMATION_H_

#include "identify_marker.h"

struct PoseInformation {
	Mat Pos;
	vector<Point3f> tracking_points;
};

class PoseEstimation {
public:
	PoseInformation poseEstimationStereo(vector<corner_pos_with_ID> corner_set_left, vector<corner_pos_with_ID> corner_set_right, Mat IntrinsicCam, Mat extrinsicCam, float model_3D);
	PoseInformation poseEstimationMono(vector<corner_pos_with_ID> corner_set, Mat IntrinsicCam, Mat DistCoeff, float(*model_3D)[3]);

	PoseInformation Pose6D;

private:
	Mat rvec, tvec, R;
	vector<Point3f> world_points;
	vector<Point2f> image_points;
};

#endif