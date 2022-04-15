#include "pose_estimation.h"

PoseInformation PoseEstimation::poseEstimationStereo(vector<corner_pos_with_ID> corner_set_left, vector<corner_pos_with_ID> corner_set_right, Mat IntrinsicCam, Mat extrinsicCam, float model_3D)
{
	return Pose6D;
}

PoseInformation PoseEstimation::poseEstimationMono(vector<corner_pos_with_ID> corner_set, Mat IntrinsicCam, Mat DistCoeff, float (*model_3D)[3])
{
	for (int i = 0; i < corner_set.size(); i++) {
		world_points.push_back(Point3f(model_3D[corner_set[i].ID][0], model_3D[corner_set[i].ID][1], model_3D[corner_set[i].ID][2]));
		image_points.push_back(corner_set[i].subpixel_pos);
	}
	solvePnP(world_points, image_points, IntrinsicCam, DistCoeff, rvec, tvec, false, SOLVEPNP_EPNP);
	Rodrigues(rvec, R);

	Pose6D.Pos = R;
	Pose6D.tracking_points.push_back(Point3f(0, 0, 0));

	return Pose6D;
}
