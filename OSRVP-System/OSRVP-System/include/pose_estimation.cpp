#include "pose_estimation.h"
#include "ceres/rotation.h"

using namespace cv;

struct SnavelyReprojectionError
{
	cv::Point2d observed;
	CameraParams cam;
	cv::Point3d point_ID;

	SnavelyReprojectionError(Point2d observation, CameraParams cam, Point3d point_ID) :observed(observation), cam(cam), point_ID(point_ID) {}

	template <typename T>
	bool operator()(const T* const rotation,
		const T* const translation,
		T* residuals)const {
		T predictions[2], pos_proj[3], pos_proj1[3], pos_world[3], rot_cam[3];
		Mat rvec_cam;
		
		pos_world[0] = T(point_ID.x);
		pos_world[1] = T(point_ID.y);
		pos_world[2] = T(point_ID.z);
		AngleAxisRotatePoint(rotation, pos_world, pos_proj1);

		pos_proj1[0] += translation[0];
		pos_proj1[1] += translation[1];
		pos_proj1[2] += translation[2];

		Rodrigues(cam.Rotation, rvec_cam);
		rot_cam[0] = T(rvec_cam.at<float>(0, 0));
		rot_cam[1] = T(rvec_cam.at<float>(1, 0));
		rot_cam[2] = T(rvec_cam.at<float>(2, 0));
		AngleAxisRotatePoint(rot_cam, pos_proj1, pos_proj);

		pos_proj[0] += T(cam.Translation.at<float>(0, 0));
		pos_proj[1] += T(cam.Translation.at<float>(1, 0));
		pos_proj[2] += T(cam.Translation.at<float>(2, 0));

		const T fx = T(cam.Intrinsic.at<float>(0, 0));
		const T fy = T(cam.Intrinsic.at<float>(1, 1));
		const T cx = T(cam.Intrinsic.at<float>(0, 2));
		const T cy = T(cam.Intrinsic.at<float>(1, 2));

		predictions[0] = fx * (pos_proj[0] / pos_proj[2]) + cx;
		predictions[1] = fy * (pos_proj[1] / pos_proj[2]) + cy;

		residuals[0] = predictions[0] - T(observed.x);
		residuals[1] = predictions[1] - T(observed.y);

		return true;
	}

	static ceres::CostFunction* Create(Point2d observed, CameraParams cam, Point3d point_ID) {
		return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 3, 3>(
			new SnavelyReprojectionError(observed, cam, point_ID)));
	}	

};

PoseEstimation::PoseEstimation()
{
	using ceres::CostFunction;
	using ceres::AutoDiffCostFunction;
	using ceres::Problem;
	using ceres::Solve;
	using ceres::Solver;

}

PoseEstimation::~PoseEstimation()
{
}

PoseInformation PoseEstimation::poseEstimation(vector<vector<corner_pos_with_ID>> corner_set, vector<CameraParams> camera_parameters, float(*model_3D)[3], unsigned int camera_num)
{
	Pose6D.recovery = false;
	for (int i = 0; i < camera_num; i++) {
		if (corner_set[i].size() > 8) {
			enough_number[number_enough_corners++] = i;
		}
	}
	if (number_enough_corners == 0) return Pose6D;
	if (number_enough_corners == 1) poseEstimationMono(corner_set, camera_parameters, model_3D);
	if (number_enough_corners == 2) poseEstimationStereo(corner_set, camera_parameters, model_3D);

	if (Pose6D.recovery)  bundleAdjustment(corner_set, camera_parameters, model_3D);

	return Pose6D;
}

void PoseEstimation::poseEstimationStereo(vector<vector<corner_pos_with_ID>> corner_set, vector<CameraParams> camera_parameters, float(*model_3D)[3])
{
	registrated_point_cnt = 0;
	for (int i = 0; i < corner_set[enough_number[0]].size(); i++)
		for (int j = 0; j < corner_set[enough_number[1]].size(); j++) {
			if ((corner_set[enough_number[0]][i].ID == corner_set[enough_number[1]][j].ID) && (model_3D[corner_set[enough_number[0]][i].ID][2] - 0.0 > 1e-2)) {
				imgpts1.push_back(corner_set[enough_number[0]][i].subpixel_pos);
				imgpts2.push_back(corner_set[enough_number[1]][j].subpixel_pos);
				pts1.push_back(Point3f(model_3D[corner_set[enough_number[0]][i].ID][0], model_3D[corner_set[enough_number[0]][i].ID][1], model_3D[corner_set[enough_number[0]][i].ID][2]));
				registrated_point_cnt++;
				continue;
			}
		}
	if (registrated_point_cnt > 8) {
		triangulation(imgpts1, imgpts2, pts2, camera_parameters[enough_number[0]], camera_parameters[enough_number[1]]);
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
				
		string filePath = "F:\\OSRVP-System\\OSRVP-System\\OSRVP-System\\Data\\";
		string cameraParametersName = filePath + "cameraParams.yml";
		FileStorage fs(cameraParametersName, FileStorage::READ);

		Pose6D.rotation = rvec;
		Pose6D.translation = tvec;
		int tracking_number;
		fs["trackingNumber"] >> tracking_number;
		string TrackingPoint = "TrackingPoint";
		Mat trackingPoint;
		for (int i = 0; i < tracking_number; i++) {
			string TrackingPointi = TrackingPoint + to_string(i + 1);
			fs[TrackingPointi] >> trackingPoint;
			Pose6D.tracking_points.push_back(Point3f(trackingPoint.at<float>(0, 0), trackingPoint.at<float>(1, 0), trackingPoint.at<float>(2, 0)));
		}	
		Pose6D.recovery = true;

		//Print into files
		Mat tracking_point(3, 1, CV_32FC1);
		tracking_point.at<float>(0, 0) = Pose6D.tracking_points[0].x;
		tracking_point.at<float>(1, 0) = Pose6D.tracking_points[0].y;
		tracking_point.at<float>(2, 0) = Pose6D.tracking_points[0].z;
		char fname[256];
		sprintf(fname, "rot1.txt");
		ofstream Files;
		Files.open(fname, ios::app);
		Files << R << endl;
		Files.close();

		sprintf(fname, "end1.txt");
		Files.open(fname, ios::app);
		Files << R * tracking_point + Pose6D.translation << endl;
		Files.close();

		sprintf(fname, "trans1.txt");
		Files.open(fname, ios::app);
		Files << Pose6D.translation << endl;
		Files.close();

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
		//cout << reprojection_error / pts1.size() << endl;
	}
}

void PoseEstimation::poseEstimationMono(vector<vector<corner_pos_with_ID>> corner_set, vector<CameraParams> camera_parameters, float (*model_3D)[3])
{
	for (int i = 0; i < corner_set[enough_number[0]].size(); i++) {
		world_points.push_back(Point3f(model_3D[corner_set[enough_number[0]][i].ID][0], model_3D[corner_set[enough_number[0]][i].ID][1], model_3D[corner_set[enough_number[0]][i].ID][2]));
		image_points.push_back(corner_set[enough_number[0]][i].subpixel_pos);
	}
	//undistortPoints(image_points, image_points, camera_parameters[enough_number[0]].Intrinsic, camera_parameters[enough_number[0]].Distortion, noArray(), camera_parameters[enough_number[0]].Intrinsic);

	solvePnPRansac(world_points, image_points, camera_parameters[enough_number[0]].Intrinsic, camera_parameters[enough_number[0]].Distortion, rvec, tvec, false, 200, 0.3, 0.8, noArray(), SOLVEPNP_EPNP);
	rvec.convertTo(rvec, CV_32FC1);
	tvec.convertTo(tvec, CV_32FC1);
	invert(camera_parameters[enough_number[0]].Rotation, Rotcam);
	Rodrigues(rvec, R);
	R = Rotcam * R;
	Rodrigues(R, rvec);

	tvec = Rotcam * tvec - camera_parameters[enough_number[0]].Translation;
	Pose6D.rotation = rvec;
	Pose6D.translation = tvec;

	string filePath = "F:\\OSRVP-System\\OSRVP-System\\OSRVP-System\\Data\\";
	string cameraParametersName = filePath + "cameraParams.yml";
	FileStorage fs(cameraParametersName, FileStorage::READ);

	int tracking_number;
	fs["trackingNumber"] >> tracking_number;
	string TrackingPoint = "TrackingPoint";
	Mat trackingPoint;
	for (int i = 0; i < tracking_number; i++) {
		string TrackingPointi = TrackingPoint + to_string(i + 1);
		fs[TrackingPointi] >> trackingPoint;
		Pose6D.tracking_points.push_back(Point3f(trackingPoint.at<float>(0, 0), trackingPoint.at<float>(1, 0), trackingPoint.at<float>(2, 0)));
	}

	vector<Point2f> imagePoints;
	projectPoints(world_points, rvec, tvec, camera_parameters[enough_number[0]].Intrinsic, camera_parameters[enough_number[0]].Distortion, imagePoints);

	float reprojection_error = 0;
	for (int i = 0; i < imagePoints.size(); i++) {
		reprojection_error += sqrt((imagePoints[i].x - image_points[i].x) * (imagePoints[i].x - image_points[i].x) + (imagePoints[i].y - image_points[i].y) * (imagePoints[i].y - image_points[i].y));
	}		
	//cout << "Origin RE: " << reprojection_error / imagePoints.size() << endl;
	
	Pose6D.recovery = true;

	//Print into files
	/*
	Mat tracking_point(3, 1, CV_32FC1);
	tracking_point.at<float>(0, 0) = Pose6D.tracking_points[0].x;
	tracking_point.at<float>(1, 0) = Pose6D.tracking_points[0].y;
	tracking_point.at<float>(2, 0) = Pose6D.tracking_points[0].z;
	char fname[256];
	sprintf(fname, "rot1.txt");
	ofstream Files;
	Files.open(fname, ios::app);
	Files << R << endl;
	Files.close();

	sprintf(fname, "end1.txt");
	Files.open(fname, ios::app);
	Files << R * tracking_point + tvec << endl;
	Files.close();

	sprintf(fname, "trans1.txt");
	Files.open(fname, ios::app);
	Files << tvec << endl;
	Files.close();
	*/
}

void PoseEstimation::bundleAdjustment(vector<vector<corner_pos_with_ID>> corner_set, vector<CameraParams> camera_parameters, float(*model_3D)[3])
{
	rot[0] = Pose6D.rotation.at<float>(0, 0);
	rot[1] = Pose6D.rotation.at<float>(1, 0);
	rot[2] = Pose6D.rotation.at<float>(2, 0);
	trans[0] = Pose6D.translation.at<float>(0, 0);
	trans[1] = Pose6D.translation.at<float>(1, 0);
	trans[2] = Pose6D.translation.at<float>(2, 0);

	buildProblem(&problem, corner_set, camera_parameters, model_3D);

	Solver::Options options;
	options.linear_solver_type = DENSE_SCHUR;
	options.gradient_tolerance = 1e-16;
	options.function_tolerance = 1e-16;
	options.parameter_tolerance = 1e-10;
	Solver::Summary summary;

	Solve(options, &problem, &summary);
	//std::cout << summary.BriefReport() << "\n";

	Pose6D.rotation.at<float>(0, 0) = rot[0];
	Pose6D.rotation.at<float>(1, 0) = rot[1];
	Pose6D.rotation.at<float>(2, 0) = rot[2];
	Pose6D.translation.at<float>(0, 0) = trans[0];
	Pose6D.translation.at<float>(1, 0) = trans[1];
	Pose6D.translation.at<float>(2, 0) = trans[2];

	world_points.clear();
	image_points.clear();
	for (int i = 0; i < corner_set[enough_number[0]].size(); i++) {
		world_points.push_back(Point3f(model_3D[corner_set[enough_number[0]][i].ID][0], model_3D[corner_set[enough_number[0]][i].ID][1], model_3D[corner_set[enough_number[0]][i].ID][2]));
		image_points.push_back(corner_set[enough_number[0]][i].subpixel_pos);
	}
	vector<Point2f> imagePoints;
	projectPoints(world_points, Pose6D.rotation, Pose6D.translation, camera_parameters[enough_number[0]].Intrinsic, camera_parameters[enough_number[0]].Distortion, imagePoints);

	float reprojection_error = 0;
	for (int i = 0; i < imagePoints.size(); i++) {
		reprojection_error += sqrt((imagePoints[i].x - image_points[i].x) * (imagePoints[i].x - image_points[i].x) + (imagePoints[i].y - image_points[i].y) * (imagePoints[i].y - image_points[i].y));
	}
	//cout << "After BA RE: " << reprojection_error / imagePoints.size() << endl;

	//Print into files
	Mat R;
	Mat tracking_point(3, 1, CV_32FC1);
	tracking_point.at<float>(0, 0) = Pose6D.tracking_points[0].x;
	tracking_point.at<float>(1, 0) = Pose6D.tracking_points[0].y;
	tracking_point.at<float>(2, 0) = Pose6D.tracking_points[0].z;
	Rodrigues(Pose6D.rotation, R);
	char fname[256];
	sprintf(fname, "rot2.txt");
	ofstream Files;
	Files.open(fname, ios::app);
	Files << R << endl;
	Files.close();

	sprintf(fname, "end2.txt");
	Files.open(fname, ios::app);
	Files << R * tracking_point + Pose6D.translation << endl;
	Files.close();

	sprintf(fname, "trans2.txt");
	Files.open(fname, ios::app);
	Files << Pose6D.translation << endl;
	Files.close();
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

void PoseEstimation::buildProblem(Problem* problem, vector<vector<corner_pos_with_ID>> corner_set, vector<CameraParams> camera_parameters, float(*model_3D)[3]) {
	rot[0] = Pose6D.rotation.at<float>(0, 0);
	rot[1] = Pose6D.rotation.at<float>(1, 0);
	rot[2] = Pose6D.rotation.at<float>(2, 0);
	trans[0] = Pose6D.translation.at<float>(0, 0);
	trans[1] = Pose6D.translation.at<float>(1, 0);
	trans[2] = Pose6D.translation.at<float>(2, 0);

	for (int i = 0; i < number_enough_corners; ++i) {
		for (int j = 0; j < corner_set[i].size(); j++) {
			CostFunction* cost_function;
			cost_function = SnavelyReprojectionError::Create((Point2d)corner_set[i][j].subpixel_pos, camera_parameters[enough_number[i]], Point3d(model_3D[corner_set[i][j].ID][0], model_3D[corner_set[i][j].ID][1], model_3D[corner_set[i][j].ID][2]));
			problem->AddResidualBlock(cost_function, NULL, rot, trans);
		}
	}
}

void CamProjectionWithDistortion(const double* const rot, const double* const trans, CameraParams cam, double* point, double* predictions)
{
	cv::Mat R, point_cam;
	cv::Mat rvec = cv::Mat::zeros(3, 1, CV_32FC1);
	cv::Mat tvec = cv::Mat::zeros(3, 1, CV_32FC1);
	cv::Mat point_world = cv::Mat::zeros(3, 1, CV_32FC1);
	vector<cv::Point2f> imagePoint;
	vector<cv::Point3f> worldPoint;
	rvec.at<float>(0, 0) = rot[0];
	rvec.at<float>(1, 0) = rot[1];
	rvec.at<float>(2, 0) = rot[2];
	tvec.at<float>(0, 0) = trans[0];
	tvec.at<float>(1, 0) = trans[1];
	tvec.at<float>(2, 0) = trans[2];
	point_world.at<float>(0, 0) = point[0];
	point_world.at<float>(1, 0) = point[1];
	point_world.at<float>(2, 0) = point[2];
	//cout << point[0] << ' ' << point[1] << ' ' << point[2] << endl;

	Rodrigues(rvec, R);
	point_cam = R * point_world + tvec;
	Rodrigues(cam.Rotation, rvec);
	imagePoint.clear();

	projectPoints(point_cam, rvec, cam.Translation, cam.Intrinsic, cam.Distortion, imagePoint);
	predictions[0] = imagePoint[0].x;
	predictions[1] = imagePoint[0].y;
}
