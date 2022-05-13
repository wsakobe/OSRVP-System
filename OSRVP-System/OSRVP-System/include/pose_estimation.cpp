#include "pose_estimation.h"

using namespace cv;

PoseEstimation::PoseEstimation()
{
	//using ceres::AutoDiffCostFunction;
	//using ceres::CostFunction;
	//using ceres::Problem;
	//using ceres::Solve;
	//using ceres::Solver;
}

PoseEstimation::~PoseEstimation()
{
}

PoseInformation PoseEstimation::poseEstimation(vector<vector<corner_pos_with_ID>> corner_set, vector<CameraParams> camera_parameters, float(*model_3D)[3], unsigned int camera_num)
{
	int number_enough_corners = 0;
	int enough_number[5] = { 0 };
	Pose6D.recovery = false;
	for (int i = 0; i < camera_num; i++) {
		if (corner_set[i].size() > 8) {
			enough_number[number_enough_corners++] = i;
		}
	}
	if (number_enough_corners == 0) return Pose6D;
	if (number_enough_corners == 1) poseEstimationMono(corner_set, camera_parameters, model_3D, enough_number[0]);
	if (number_enough_corners == 2) poseEstimationStereo(corner_set, camera_parameters, model_3D, enough_number);



	return Pose6D;
}

void PoseEstimation::poseEstimationStereo(vector<vector<corner_pos_with_ID>> corner_set, vector<CameraParams> camera_parameters, float(*model_3D)[3], int camera_number[5])
{
	registrated_point_cnt = 0;
	for (int i = 0; i < corner_set[camera_number[0]].size(); i++)
		for (int j = 0; j < corner_set[camera_number[1]].size(); j++) {
			if ((corner_set[camera_number[0]][i].ID == corner_set[camera_number[1]][j].ID) && (model_3D[corner_set[camera_number[0]][i].ID][2] - 0.0 > 1e-2)) {
				imgpts1.push_back(corner_set[camera_number[0]][i].subpixel_pos);
				imgpts2.push_back(corner_set[camera_number[1]][j].subpixel_pos);
				pts1.push_back(Point3f(model_3D[corner_set[camera_number[0]][i].ID][0], model_3D[corner_set[camera_number[0]][i].ID][1], model_3D[corner_set[camera_number[0]][i].ID][2]));
				registrated_point_cnt++;
				continue;
			}
		}
	if (registrated_point_cnt > 8) {
		triangulation(imgpts1, imgpts2, pts2, camera_parameters[camera_number[0]], camera_parameters[camera_number[1]]);
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
		
		/*
		char fname[256];
		sprintf(fname, "rot.txt");
		ofstream Files;
		Files.open(fname, ios::app);
		Files << R << endl;
		
		Files.close();

		sprintf(fname, "end.txt");
		Files.open(fname, ios::app);
		Files << R * end_effector + tvec << endl;
		Files.close();

		sprintf(fname, "trans.txt");
		Files.open(fname, ios::app);
		Files << tvec << endl;
		Files.close();
		*/
		string filePath = "F:\\OSRVP-System\\OSRVP-System\\OSRVP-System\\Data\\";
		string cameraParametersName = filePath + "cameraParams.yml";
		FileStorage fs(cameraParametersName, FileStorage::READ);

		Pose6D.rotation = rvec;
		Pose6D.translation = tvec;
		fs["TrackingPoint"] >> end_effector;
		Pose6D.tracking_points.push_back(Point3d(end_effector.at<float>(0, 0), end_effector.at<float>(1, 0), end_effector.at <float> (2, 0)));
		Pose6D.recovery = true;

		float reprojection_error = 0;
		Eigen::Matrix<float, 3, 3> R_eigen;
		Eigen::Matrix<float, 3, 1> T_eigen;
		cv::cv2eigen(R, R_eigen);
		cv::cv2eigen(tvec, T_eigen);
		for (int i = 0; i < pts1.size(); i++) {
			Eigen::MatrixXf PT1(3, 1);
			PT1 << pts1[i].x, pts1[i].y, pts1[i].z;
			Eigen::MatrixXf PT2(3, 1);
			PT2 << pts2[i].x, pts2[i].y, pts2[i].z;
			PT1 = R_eigen * PT1 + T_eigen;
			reprojection_error += (PT1 - PT2).norm();
		}			
		cout << reprojection_error / pts1.size() << endl;
	}
}

void PoseEstimation::poseEstimationMono(vector<vector<corner_pos_with_ID>> corner_set, vector<CameraParams> camera_parameters, float (*model_3D)[3], int camera_number)
{
	for (int i = 0; i < corner_set[camera_number].size(); i++) {
		world_points.push_back(Point3f(model_3D[corner_set[camera_number][i].ID][0], model_3D[corner_set[camera_number][i].ID][1], model_3D[corner_set[camera_number][i].ID][2]));
		image_points.push_back(corner_set[camera_number][i].subpixel_pos);
	}
	undistortPoints(image_points, image_points, camera_parameters[camera_number].Intrinsic, camera_parameters[camera_number].Distortion, noArray(), camera_parameters[camera_number].Intrinsic);

	solvePnPRansac(world_points, image_points, camera_parameters[camera_number].Intrinsic, camera_parameters[camera_number].Distortion, rvec, tvec, false, 200, 0.5, 0.9, noArray(), SOLVEPNP_EPNP);
	rvec.convertTo(rvec, CV_32FC1);
	tvec.convertTo(tvec, CV_32FC1);
	Rodrigues(rvec, R);
	//cout << R << endl;

	/*
	char fname[256];
	sprintf(fname, "rot.txt");
	ofstream Files;
	Files.open(fname, ios::app);
	Files << R << endl;
	Files.close();

	sprintf(fname, "end.txt");
	Files.open(fname, ios::app);
	Files << R * end_effector + tvec << endl;
	Files.close();

	sprintf(fname, "trans.txt");
	Files.open(fname, ios::app);
	Files << tvec << endl;
	Files.close();
	*/
	Pose6D.rotation = rvec;
	Pose6D.translation = tvec;
	Pose6D.tracking_points.push_back(Point3d(end_effector.at<float>(0, 0), end_effector.at<float>(1, 0), end_effector.at<float>(2, 0)));

	vector<Point2f> imagePoints;
	projectPoints(world_points, rvec, tvec, camera_parameters[camera_number].Intrinsic, camera_parameters[camera_number].Distortion, imagePoints);

	float reprojection_error = 0;
	for (int i = 0; i < imagePoints.size(); i++)
		reprojection_error += sqrt((imagePoints[i].x - image_points[i].x) * (imagePoints[i].x - image_points[i].x) + (imagePoints[i].y - image_points[i].y) * (imagePoints[i].y - image_points[i].y));
	cout << reprojection_error / imagePoints.size() << endl;
	
	Pose6D.recovery = true;
}

void PoseEstimation::bundleAdjustment(PoseInformation Pose6D, vector<vector<corner_pos_with_ID>> corner_set, vector<CameraParams> camera_parameters, float(*model_3D)[3], int camera_number[5])
{

}

void PoseEstimation::triangulation(const std::vector<Point2f>& points_left, const std::vector<Point2f>& points_right, std::vector<Point3f>& points, CameraParams camera_parameter1, CameraParams camera_parameter2)
{
	if (!points_left.size())
		return;

	Mat mLeftRT = Mat(3, 4, CV_32FC1);    //左相机M矩阵
	hconcat(camera_parameter1.Rotation, camera_parameter1.Translation, mLeftRT);
	Mat mLeftM = camera_parameter1.Intrinsic * mLeftRT;

	Mat mRightRT = Mat(3, 4, CV_32FC1);   //右相机M矩阵
	hconcat(camera_parameter2.Rotation, camera_parameter2.Translation, mRightRT);
	Mat mRightM = camera_parameter2.Intrinsic * mRightRT;

	undistortPoints(points_left, points_left, camera_parameter1.Intrinsic, camera_parameter1.Distortion, noArray(), camera_parameter1.Intrinsic);
	undistortPoints(points_right, points_right, camera_parameter2.Intrinsic, camera_parameter2.Distortion, noArray(), camera_parameter2.Intrinsic);

	for (int i = 0; i < points_left.size(); i++) {
		// 将像素坐标转换至相机坐标
		pts_1.push_back(pixel2cam(points_left[i],  camera_parameter1.Intrinsic));
		pts_2.push_back(pixel2cam(points_right[i], camera_parameter2.Intrinsic));
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
		(p.x - K.at<float>(0, 2)) / K.at<float>(0, 0),
		(p.y - K.at<float>(1, 2)) / K.at<float>(1, 1)
	);
}