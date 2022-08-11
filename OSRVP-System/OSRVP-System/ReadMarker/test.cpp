#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Core>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <sophus/se3.hpp>
#include <chrono>

using namespace std;
using namespace cv;

// 像素坐标转相机归一化坐标
Point2d pixel2cam(const Point2d& p, const Mat& K);

// BA by g2o
typedef vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> VecVector2d;
typedef vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> VecVector3d;

// BA by gauss-newton
void bundleAdjustmentGaussNewton(
    const VecVector3d& points_3d,
    const VecVector2d& points_2d,
    const Mat& K,
    Sophus::SE3d& pose
);

int main(int argc, char** argv) {
    const char* filename1 = "23point.txt";

    fstream inFile;

    inFile.open(filename1, ios::in);

    vector<vector<double>> points_2d;
    vector<vector<double>> xyzpoints;
    points_2d.resize(24);
    for (size_t i = 0; i < 24; i++)
        points_2d[i].resize(2);
    xyzpoints.resize(24);
    for (size_t i = 0; i < 24; i++)
        xyzpoints[i].resize(3);

    char buffer[100];
    size_t i = 0;
    while (true) {
        inFile.getline(buffer, 100, '\n');
        sscanf_s(buffer, "[%lf, %lf, %lf][%lf, %lf]", &xyzpoints[i][0], &xyzpoints[i][1], &xyzpoints[i][2], &points_2d[i][0], &points_2d[i][1]);
        i++;
        if (i == 24) break;
    }

    vector<Point3f> p3d;
    vector<Point2f> p2d;
    for (size_t i = 0; i < 24; i++) {
        p2d.push_back(Point2f(points_2d[i][0], points_2d[i][1]));
    }

    for (size_t i = 0; i < 24; i++) {
        p3d.push_back(Point3f(xyzpoints[i][0], xyzpoints[i][1], xyzpoints[i][2]));
    }
    Mat K = (Mat_<double>(3, 3) << 4243.15371428429, 0, 939.124364630470, 0, 4242.74219958624, 628.056169563970, 0, 0, 1);
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    Mat r, t;
    solvePnP(p3d, p2d, K, Mat(), r, t, false, SOLVEPNP_EPNP); // 调用OpenCV 的 PnP 求解，可选择EPNP，DLS等方法
    Mat R;
    cv::Rodrigues(r, R); // r为旋转向量形式，用Rodrigues公式转换为矩阵
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    //cout << "solve pnp in opencv cost time: " << time_used.count() << " seconds." << endl;

    cout << "R=" << endl << R << endl;
    cout << "t=" << endl << t << endl;

    VecVector3d pts_3d_eigen;
    VecVector2d pts_2d_eigen;
    for (size_t i = 0; i < p3d.size(); ++i) {
        pts_3d_eigen.push_back(Eigen::Vector3d(p3d[i].x, p3d[i].y, p3d[i].z));
        pts_2d_eigen.push_back(Eigen::Vector2d(p2d[i].x, p2d[i].y));
    }

    cout << "calling bundle adjustment by gauss newton" << endl;
    Eigen::Matrix3d RR;
    Eigen::Vector3d TT;
    RR << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2);
    TT << t.at<double>(0, 0), t.at<double>(1, 0), t.at<double>(2, 0);
    Sophus::SE3d pose_gn(RR, TT); cout << "init pose by g-n: \n" << pose_gn.matrix() << endl;
    // BA 前重投影误差
    Eigen::Vector2d e[101];
    double fx = K.at<double>(0, 0);
    double fy = K.at<double>(1, 1);
    double cx = K.at<double>(0, 2);
    double cy = K.at<double>(1, 2);
    double sum_err = 0, avg_err = 0;
    for (int i = 0; i < pts_3d_eigen.size(); i++) {
        Eigen::Vector3d pc = pose_gn * pts_3d_eigen[i];
        double inv_z = 1.0 / pc[2];
        double inv_z2 = inv_z * inv_z;
        Eigen::Vector2d proj(fx * pc[0] / pc[2] + cx, fy * pc[1] / pc[2] + cy);

        e[i] = pts_2d_eigen[i] - proj;
        sum_err += sqrt(e[i].squaredNorm());
        //cout << "reprojection error: " << sqrt(e[i].squaredNorm()) << endl;
    }
    avg_err = sum_err / pts_3d_eigen.size();
    cout << "average reprojection error before bundle adjustment: " << avg_err << endl;
    cout << endl;

    t1 = chrono::steady_clock::now();
    bundleAdjustmentGaussNewton(pts_3d_eigen, pts_2d_eigen, K, pose_gn);
    t2 = chrono::steady_clock::now();
    time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "bundle adjustment by gauss newton cost time: " << time_used.count() << " seconds." << endl;
    // BA后重投影误差
    sum_err = 0;
    for (int i = 0; i < pts_3d_eigen.size(); i++) {
        Eigen::Vector3d pc = pose_gn * pts_3d_eigen[i];
        double inv_z = 1.0 / pc[2];
        double inv_z2 = inv_z * inv_z;
        Eigen::Vector2d proj(fx * pc[0] / pc[2] + cx, fy * pc[1] / pc[2] + cy);

        e[i] = pts_2d_eigen[i] - proj;
        sum_err += sqrt(e[i].squaredNorm());
        //cout << "reprojection error: " << sqrt(e[i].squaredNorm()) << endl;
    }
    avg_err = sum_err / pts_3d_eigen.size();
    cout << "average reprojection error after bundle adjustment: " << avg_err << endl;

    return 0;
}
