#include "..\include\pose_estimation.h"
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

PoseInformation PoseEstimation::poseEstimation(vector<vector<corner_pos_with_ID>> corner_set, vector<CameraParams> camera_parameters, float(*model_3D)[3], unsigned int camera_num, bool camera_type)
{
	Pose6D.recovery = false;
	
	vector<Point2f> corners_ori, corners_undistort;
	for (int i = 0; i < camera_num; i++) {
		if (corner_set[i].size() > 5) {
			corners_ori.clear();
			corners_undistort.clear();
			for (int j = 0; j < corner_set[i].size(); j++)
				corners_ori.push_back(corner_set[i][j].subpixel_pos);
			undistortPoints(corners_ori, corners_undistort, camera_parameters[i].Intrinsic, camera_parameters[i].Distortion, noArray(), camera_parameters[i].Intrinsic);
			for (int j = 0; j < corner_set[i].size(); j++)
				corner_set[i][j].subpixel_pos = corners_undistort[j];
		}
	}
	
	if (camera_type == HikingCamera) {
		for (int i = 0; i < camera_num; i++) {
			if (corner_set[i].size() > 5) {
				enough_number[number_enough_corners++] = i;
			}
		}
		if (number_enough_corners == 0) return Pose6D;
		if (number_enough_corners == 1) poseEstimationMono(corner_set, camera_parameters, model_3D);
		if (number_enough_corners == 2) poseEstimationStereo(corner_set, camera_parameters, model_3D);

		if (Pose6D.recovery)  bundleAdjustment(corner_set, camera_parameters, model_3D);
	}
	else if (camera_type == USBCamera) {
		if (corner_set[0].size() > 3) {
			poseEstimationPlanar(corner_set, camera_parameters);
		}
	}

	return Pose6D;
}

void PoseEstimation::poseEstimationStereo(vector<vector<corner_pos_with_ID>> corner_set, vector<CameraParams> camera_parameters, float(*model_3D)[3])
{
	registrated_point_cnt = 0;
	for (int i = 0; i < corner_set[enough_number[0]].size(); i++)
		for (int j = 0; j < corner_set[enough_number[1]].size(); j++) {
			if ((corner_set[enough_number[0]][i].ID == corner_set[enough_number[1]][j].ID) && (abs(model_3D[corner_set[enough_number[0]][i].ID][2] - 0.0) > 1e-2)) {
				imgpts1.push_back(corner_set[enough_number[0]][i].subpixel_pos);
				imgpts2.push_back(corner_set[enough_number[1]][j].subpixel_pos);
				pts1.push_back(Point3f(model_3D[corner_set[enough_number[0]][i].ID][0], model_3D[corner_set[enough_number[0]][i].ID][1], model_3D[corner_set[enough_number[0]][i].ID][2]));
				registrated_point_cnt++;
				continue;
			}
		}

	// pts1 -> World frame; pts2 -> Cam frame
	if (registrated_point_cnt > 8) {
		triangulation(imgpts1, imgpts2, pts2, camera_parameters[enough_number[0]], camera_parameters[enough_number[1]]);

		Point3f p1, p2;
		for (int i = 0; i < pts1.size(); i++) {
			p1 += pts1[i];
			p2 += pts2[i];
		}
		int n = pts1.size();
		p1 = p1 / n; p2 = p2 / n;
		Mat q_cam = Mat(3, 1, CV_32FC1);
		Mat q_world = Mat(1, 3, CV_32FC1);
		Mat W = Mat::zeros(3, 3, CV_32FC1);
		Mat R, tvec, rvec;
		for (int i = 0; i < pts1.size(); i++) {
			q_cam.ptr<float>(0)[0] = pts2[i].x - p2.x;
			q_cam.ptr<float>(1)[0] = pts2[i].y - p2.y;
			q_cam.ptr<float>(2)[0] = pts2[i].z - p2.z;
			q_world.ptr<float>(0)[0] = pts1[i].x - p1.x;
			q_world.ptr<float>(0)[1] = pts1[i].y - p1.y;
			q_world.ptr<float>(0)[2] = pts1[i].z - p1.z;
			W = W + q_cam * q_world;
		}
		Mat U, S, Vt;
		SVDecomp(W, S, U, Vt);
		
		R = U * Vt;
		
		Mat pm_world = (Mat_<float>(3, 1) << p1.x, p1.y, p1.z);
		Mat pm_cam = (Mat_<float>(3, 1) << p2.x, p2.y, p2.z);
		
		tvec = pm_cam - R * pm_world;
		
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
		Files << format(R, cv::Formatter::FMT_CSV) << endl;
		Files.close();

		sprintf(fname, "end1.txt");
		Files.open(fname, ios::app);
		Files << format(R * tracking_point + tvec, cv::Formatter::FMT_CSV) << endl;
		Files.close();

		sprintf(fname, "trans1.txt");
		Files.open(fname, ios::app);
		Files << format(tvec, cv::Formatter::FMT_CSV) << endl;
		Files.close();
		
		world_points.clear();
		image_points.clear();
		for (int i = 0; i < corner_set[enough_number[0]].size(); i++) {
			if (abs(model_3D[corner_set[enough_number[0]][i].ID][2] - 0.0) > 1e-2) {
				world_points.push_back(Point3f(model_3D[corner_set[enough_number[0]][i].ID][0], model_3D[corner_set[enough_number[0]][i].ID][1], model_3D[corner_set[enough_number[0]][i].ID][2]));
				image_points.push_back(corner_set[enough_number[0]][i].subpixel_pos);
			}
		}
		vector<Point2f> imagePoints;
		projectPoints(world_points, Pose6D.rotation, Pose6D.translation, camera_parameters[enough_number[0]].Intrinsic, cv::Mat::zeros(5, 1, CV_32FC1), imagePoints);
		float reprojection_error = 0;
		for (int i = 0; i < imagePoints.size(); i++) {
			reprojection_error += sqrt((imagePoints[i].x - image_points[i].x) * (imagePoints[i].x - image_points[i].x) + (imagePoints[i].y - image_points[i].y) * (imagePoints[i].y - image_points[i].y));
		}
		cout << "ICP RPE: " << reprojection_error / imagePoints.size();
	}
}

void PoseEstimation::poseEstimationMono(vector<vector<corner_pos_with_ID>> corner_set, vector<CameraParams> camera_parameters, float (*model_3D)[3])
{
	for (int i = 0; i < corner_set[enough_number[0]].size(); i++) {
		if (abs(model_3D[corner_set[enough_number[0]][i].ID][0] - 0.0) > 1e-2) {
			world_points.push_back(Point3f(model_3D[corner_set[enough_number[0]][i].ID][0], model_3D[corner_set[enough_number[0]][i].ID][1], model_3D[corner_set[enough_number[0]][i].ID][2]));
			image_points.push_back(corner_set[enough_number[0]][i].subpixel_pos);
		}
	}
	solvePnPRansac(world_points, image_points, camera_parameters[enough_number[0]].Intrinsic, cv::Mat::zeros(5, 1, CV_32FC1), rvec, tvec, false, 200, 0.3, 0.8, noArray(), SOLVEPNP_EPNP);
	
	rvec.convertTo(rvec, CV_32FC1);
	tvec.convertTo(tvec, CV_32FC1);
	Rodrigues(rvec, R);
	R = camera_parameters[enough_number[0]].Rotation.t() * R;
	Rodrigues(R, rvec);
	tvec = camera_parameters[enough_number[0]].Rotation.t() * tvec - camera_parameters[enough_number[0]].Rotation.t() * camera_parameters[enough_number[0]].Translation;
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
	projectPoints(world_points, rvec, tvec, camera_parameters[enough_number[0]].Intrinsic, cv::Mat::zeros(5, 1, CV_32FC1), imagePoints);
	float reprojection_error = 0;
	for (int i = 0; i < imagePoints.size(); i++) {
		reprojection_error += sqrt((imagePoints[i].x - image_points[i].x) * (imagePoints[i].x - image_points[i].x) + (imagePoints[i].y - image_points[i].y) * (imagePoints[i].y - image_points[i].y));
	}		
	cout << "EPnP RPE: " << reprojection_error / imagePoints.size();
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
	Files << format(R, cv::Formatter::FMT_CSV) << endl;
	Files.close();

	sprintf(fname, "end1.txt");
	Files.open(fname, ios::app);
	Files << format(R * tracking_point + tvec, cv::Formatter::FMT_CSV) << endl;
	Files.close();

	sprintf(fname, "trans1.txt");
	Files.open(fname, ios::app);
	Files << format(tvec, cv::Formatter::FMT_CSV) << endl;
	Files.close();
}

void PoseEstimation::poseEstimationPlanar(vector<vector<corner_pos_with_ID>> corner_set, vector<CameraParams> camera_parameters)
{
	vector<Point2f> imagePoints, imagePoints_unit, planarModel;
	for (int i = 0; i < corner_set[0].size(); i++) {
		imagePoints.push_back(corner_set[0][i].subpixel_pos);
		imagePoints_unit.push_back(pixel2cam(imagePoints[i], camera_parameters[0].Intrinsic));
		planarModel.push_back(Point2f((corner_set[0][i].ID - 1) % 15 * 2, (corner_set[0][i].ID - 1) / 15 * 2));
	}
	
	Mat H = findHomography(planarModel, imagePoints_unit);
	
	// Normalization to ensure that ||c1|| = 1
	double norm = sqrt(H.at<double>(0, 0) * H.at<double>(0, 0) +
		H.at<double>(1, 0) * H.at<double>(1, 0) +
		H.at<double>(2, 0) * H.at<double>(2, 0));
	H /= norm;
	Mat c1 = H.col(0);
	Mat c2 = H.col(1);
	Mat c3 = c1.cross(c2);
	Mat tvec = H.col(2);
	Mat R(3, 3, CV_64F);
	for (int i = 0; i < 3; i++)
	{
		R.at<double>(i, 0) = c1.at<double>(i, 0);
		R.at<double>(i, 1) = c2.at<double>(i, 0);
		R.at<double>(i, 2) = c3.at<double>(i, 0);
	}

	Mat_<double> W, U, Vt;
	SVDecomp(R, W, U, Vt);
	R = U * Vt;
	double det = determinant(R);
	if (det < 0)
	{
		Vt.at<double>(2, 0) *= -1;
		Vt.at<double>(2, 1) *= -1;
		Vt.at<double>(2, 2) *= -1;
		R = U * Vt;
	}
	cout << "R (after polar decomposition):\n" << R << endl;
	Mat rvec;
	Rodrigues(R, rvec);

	Pose6D.rotation = rvec;
	Pose6D.translation = tvec;
	Pose6D.recovery = true;

	vector<Point2f> image_points;
	vector<Point3f> planarPoints;
	for (int i = 0; i < imagePoints.size(); i++) {
		planarPoints.push_back(Point3f(planarModel[i].x, planarModel[i].y, 0));
	}
	projectPoints(planarPoints, rvec, tvec, camera_parameters[0].Intrinsic, cv::Mat::zeros(5, 1, CV_32FC1), image_points);
	
	float reprojection_error = 0;
	for (int i = 0; i < imagePoints.size(); i++) {
		reprojection_error += sqrt((imagePoints[i].x - image_points[i].x) * (imagePoints[i].x - image_points[i].x) + (imagePoints[i].y - image_points[i].y) * (imagePoints[i].y - image_points[i].y));
	}
	cout << "Planar RPE: " << reprojection_error / imagePoints.size() << endl;
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
	options.gradient_tolerance = 1e-15;
	options.function_tolerance = 1e-15;
	options.parameter_tolerance = 1e-10;
	Solver::Summary summary;

	Solve(options, &problem, &summary);
	std::cout << summary.BriefReport() << "\n";

	Pose6D.rotation.at<float>(0, 0) = rot[0];
	Pose6D.rotation.at<float>(1, 0) = rot[1];
	Pose6D.rotation.at<float>(2, 0) = rot[2];
	Pose6D.translation.at<float>(0, 0) = trans[0];
	Pose6D.translation.at<float>(1, 0) = trans[1];
	Pose6D.translation.at<float>(2, 0) = trans[2];

	world_points.clear();
	image_points.clear();
	for (int i = 0; i < corner_set[enough_number[0]].size(); i++) {
		if (abs(model_3D[corner_set[enough_number[0]][i].ID][2] - 0.0) > 1e-2) {
			world_points.push_back(Point3f(model_3D[corner_set[enough_number[0]][i].ID][0], model_3D[corner_set[enough_number[0]][i].ID][1], model_3D[corner_set[enough_number[0]][i].ID][2]));
			image_points.push_back(corner_set[enough_number[0]][i].subpixel_pos);
		}
	}
	vector<Point2f> imagePoints;
	projectPoints(world_points, Pose6D.rotation, Pose6D.translation, camera_parameters[enough_number[0]].Intrinsic, cv::Mat::zeros(5, 1, CV_32FC1), imagePoints);
	float reprojection_error = 0;
	for (int i = 0; i < imagePoints.size(); i++) {
		reprojection_error += sqrt((imagePoints[i].x - image_points[i].x) * (imagePoints[i].x - image_points[i].x) + (imagePoints[i].y - image_points[i].y) * (imagePoints[i].y - image_points[i].y));
	}
	cout << "  After BA RPE: " << reprojection_error / imagePoints.size() << endl;
	
	//if (reprojection_error / imagePoints.size() < 0.5) {
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
		Files << format(R, cv::Formatter::FMT_CSV) << endl;
		Files.close();

		sprintf(fname, "end2.txt");
		Files.open(fname, ios::app);
		Files << format(R * tracking_point + Pose6D.translation, cv::Formatter::FMT_CSV) << endl;
		Files.close();

		sprintf(fname, "trans2.txt");
		Files.open(fname, ios::app);
		Files << format(Pose6D.translation, cv::Formatter::FMT_CSV) << endl;
		Files.close();
	//}
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

	//undistortPoints(points_left, points_left, camera_parameter1.Intrinsic, camera_parameter1.Distortion, noArray(), camera_parameter1.Intrinsic);
	//undistortPoints(points_right, points_right, camera_parameter2.Intrinsic, camera_parameter2.Distortion, noArray(), camera_parameter2.Intrinsic);

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
			Point3f p(
				x.at<float>(0, 0),
				x.at<float>(1, 0),
				x.at<float>(2, 0)
			);
			points.push_back(p);
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
			if (abs(model_3D[corner_set[i][j].ID][0] - 0.0) > 1e-2) {
				CostFunction* cost_function;
				cost_function = SnavelyReprojectionError::Create((Point2d)corner_set[i][j].subpixel_pos, camera_parameters[enough_number[i]], Point3d(model_3D[corner_set[i][j].ID][0], model_3D[corner_set[i][j].ID][1], model_3D[corner_set[i][j].ID][2]));
				problem->AddResidualBlock(cost_function, NULL, rot, trans);
			}
		}
	}
}
