#include "pose_estimation.h"

PoseInformation PoseEstimation::poseEstimationStereo(vector<corner_pos_with_ID> corner_set_left, vector<corner_pos_with_ID> corner_set_right, Mat IntrinsicCamLeft, Mat DistcoeffLeft, Mat IntrinsicCamRight, Mat DistcoeffRight, Mat Rot, Mat Trans, float(*model_3D)[3])
{
	for (int i = 0; i < corner_set_left.size(); i++)
		for (int j = 0; j < corner_set_right.size(); j++) {
			if (corner_set_left[i].ID == corner_set_right[j].ID) {
				imgpts1.push_back(corner_set_left[i].subpixel_pos);
				imgpts2.push_back(corner_set_right[j].subpixel_pos);
				pts1.push_back(Point3f(model_3D[corner_set_left[i].ID][0], model_3D[corner_set_left[i].ID][1], model_3D[corner_set_left[i].ID][2]));
				continue;
			}
		}
	if (registrated_point_cnt > 10) {
		triangulation(imgpts1, imgpts2, pts2, IntrinsicCamLeft, IntrinsicCamRight, Rot, Trans);
		Point3f p1, p2;
		for (int i = 0; i < pts1.size(); i++) {
			p1 += pts1[i];
			p2 += pts2[i];
		}
		int n = pts1.size();
		p1 = p1 / n; p2 = p2 / n;
		Mat q1 = Mat(3, 1, CV_32FC1);
		Mat q2 = Mat(1, 3, CV_32FC1);
		Mat W = Mat::zeros(3, 3, CV_32FC1);
		Mat R, tvec, rvec;
		for (int i = 0; i < pts1.size(); i++) {
			q1.ptr<float>(0)[0] = pts1[i].x - p1.x;
			q1.ptr<float>(1)[0] = pts1[i].y - p1.y;
			q1.ptr<float>(2)[0] = pts1[i].z - p1.z;
			q2.ptr<float>(0)[0] = pts2[i].x - p2.x;
			q2.ptr<float>(0)[1] = pts2[i].y - p2.y;
			q2.ptr<float>(0)[2] = pts2[i].z - p2.z;
			W = W + q1 * q2;
		}
		Mat U, S, Vt;
		SVDecomp(W, S, U, Vt);
		R = U * Vt;
		R = R.t();
		Mat pm1 = (Mat_<float>(3, 1) << p1.x, p1.y, p1.z);
		Mat pm2 = (Mat_<float>(3, 1) << p2.x, p2.y, p2.z);

		tvec = pm2 - R * pm1;

		Rodrigues(R, rvec);
	}
	return Pose6D;
}

PoseInformation PoseEstimation::poseEstimationMono(vector<corner_pos_with_ID> corner_set, Mat IntrinsicCam, Mat DistCoeff, float (*model_3D)[3])
{
	for (int i = 0; i < corner_set.size(); i++) {
		world_points.push_back(Point3f(model_3D[corner_set[i].ID][0], model_3D[corner_set[i].ID][1], model_3D[corner_set[i].ID][2]));
		image_points.push_back(corner_set[i].subpixel_pos);
	}
	solvePnPRansac(world_points, image_points, IntrinsicCam, DistCoeff, rvec, tvec, false, 100, 0.5, 0.99, noArray(), SOLVEPNP_EPNP);
	Rodrigues(rvec, R);
	//cout << R << endl;

	Mat end_effector = (Mat_<double>(3, 1) << 29.9093, 212.3799, 369);
	end_effector = R * end_effector + tvec;

	Pose6D.rotation = rvec;
	Pose6D.translation = tvec;
	Pose6D.tracking_points.push_back(Point3d(end_effector.at<double>(0, 0), end_effector.at<double>(1, 0), end_effector.at<double>(2, 0)));

	vector<Point2f> imagePoints;
	projectPoints(world_points, rvec, tvec, IntrinsicCam, DistCoeff, imagePoints);

	float reprojection_error = 0;
	for (int i = 0; i < imagePoints.size(); i++)
		reprojection_error += sqrt((imagePoints[i].x - image_points[i].x) * (imagePoints[i].x - image_points[i].x) + (imagePoints[i].y - image_points[i].y) * (imagePoints[i].y - image_points[i].y));
	//cout << reprojection_error / imagePoints.size() << endl;

	return Pose6D;
}

void PoseEstimation::triangulation(const std::vector<Point2f>& points_left, const std::vector<Point2f>& points_right, std::vector<Point3f>& points, Mat IntrinsicCamLeft, Mat IntrinsicCamRight, Mat Rot, Mat Trans)
{
	if (!points_left.size())
		return;

	Mat mLeftRotation = Mat::eye(3, 3, CV_64F);
	Mat mLeftTranslation = Mat::zeros(3, 1, CV_64F);
	Mat mLeftRT = Mat(3, 4, CV_64F);    //左相机M矩阵
	hconcat(mLeftRotation, mLeftTranslation, mLeftRT);
	Mat mLeftM = IntrinsicCamLeft * mLeftRT;

	Mat mRightRT = Mat(3, 4, CV_64F);   //右相机M矩阵
	hconcat(Rot, Trans, mRightRT);
	Mat mRightM = IntrinsicCamRight * mRightRT;

	for (int i = 0; i < points_left.size(); i++) {
		// 将像素坐标转换至相机坐标
		pts_1.push_back(pixel2cam(points_left[i], IntrinsicCamLeft));
		pts_2.push_back(pixel2cam(points_right[i], IntrinsicCamRight));
	}
	
	triangulatePoints(mLeftRT, mRightRT, pts_1, pts_2, pts_4d);

	// 转换成非齐次坐标
	if (!pts_4d.empty())
		for (int i = 0; i < pts_4d.cols; i++)
		{
			Mat x = pts_4d.col(i);
			x /= x.at<float>(3, 0); // 归一化
			Point3d p(
				x.at<float>(0, 0),
				x.at<float>(1, 0),
				x.at<float>(2, 0)
			);
			points.push_back((Point3f)p);
		}
}

Point2f PoseEstimation::pixel2cam(const Point2f& p, const Mat& K)
{
	return Point2f
	(
		(p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
		(p.y - K.at<double>(1, 2)) / K.at<double>(1, 1)
	);
}