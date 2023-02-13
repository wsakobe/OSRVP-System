#include "..\include\read_marker.h"

using namespace cv;
using namespace std;

cornerMarkerInfo ReadMarker::readMarker(Mat& image, bool cameraType, DynamicROIBox Box)
{
    cornerMarkerInfo corners;
    image_gray = image.clone();
    if (image_gray.channels() != 1) {
        cvtColor(image_gray, image_gray, COLOR_BGR2GRAY);
    }
    image_gray.convertTo(image_gray, CV_32FC1); image_gray *= 1. / 255;

    corner_pos.clear();

    ImgParams.height = image_gray.rows;
    ImgParams.width = image_gray.cols;

    //start_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

    PreFilter pF;
    candidate_corners = pF.preFilter(image_gray, number_of_corner_x_input * number_of_corner_y_input);
    // last_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    FinalElection fE;
    cornerPoints = fE.finalElection(image_gray, candidate_corners);

    ArrayOrganization arrayOrg;
    int* matrix_p = arrayOrg.delaunayTriangulation(image_gray, cornerPoints);

    IdentifyMarker identify;
    corners = identify.identifyMarker(image_gray, matrix_p, cornerPoints, value_matrix, dot_matrix, cameraType);

    for (int j = 0; j < corners.robot_marker.size(); j++)
        corners.robot_marker[j].subpixel_pos += Point2f(Box.position);
    for (int j = 0; j < corners.opener_marker.size(); j++)
        corners.opener_marker[j].subpixel_pos += Point2f(Box.position);
    for (int j = 0; j < corners.endoscope_marker.size(); j++)
        corners.endoscope_marker[j].subpixel_pos += Point2f(Box.position);

    return corners;
}

void ReadMarker::dynamicROI(FinalPoseInformation Poses, vector<DynamicROIBox>& Box, CameraParams camera_parameters, vector<ModelInfo> model_3D, bool cameraType)
{
    if (cameraType == HikingCamera) {
        if (!Poses.robot_pose.recovery) {
            if (++Box[0].lostFrame > maxLostFrame) {
                Box[0].position = Point(0, 0);
                Box[0].height = ImgParamsOri.height;
                Box[0].width = ImgParamsOri.width;
            }
            return;
        }
        axesPoints.clear();
        for (int j = 0; j < model_3D[0].corners.size(); j++) {
            axesPoints.push_back(model_3D[0].corners[j]);
        }

        cnt = 1, x_min = ImgParamsOri.width, y_min = ImgParamsOri.height, x_max = -1, y_max = -1;
        imagePoints.clear();
        Rot = Mat::zeros(3, 3, CV_32FC1);
        Rodrigues(Poses.robot_pose.rotation, Rot);
        Rot = camera_parameters.Rotation * Rot;
        rvec = Mat::zeros(3, 1, CV_32FC1);
        Rodrigues(Rot, rvec);
        projectPoints(axesPoints, rvec, camera_parameters.Rotation * Poses.robot_pose.translation + camera_parameters.Translation, camera_parameters.Intrinsic, camera_parameters.Distortion, imagePoints);

        for (int j = 0; j < imagePoints.size(); j++) {
            if (floor(imagePoints[j].x) < x_min) x_min = floor(imagePoints[j].x);
            if (floor(imagePoints[j].y) < y_min) y_min = floor(imagePoints[j].y);
            if (ceil(imagePoints[j].x) > x_max) x_max = ceil(imagePoints[j].x);
            if (ceil(imagePoints[j].y) > y_max) y_max = ceil(imagePoints[j].y);
        }
        Box[0].position = Point(max(0, x_min - BoxBorder), max(0, y_min - BoxBorder));
        Box[0].width = min(x_max - x_min + BoxBorder * 2, ImgParamsOri.width - Box[0].position.x - 1);
        Box[0].height = min(y_max - y_min + BoxBorder * 2, ImgParamsOri.height - Box[0].position.y - 1);
        Box[0].lostFrame = 0;

        if (!Poses.opener_pose.recovery) {
            if (++Box[1].lostFrame > maxLostFrame) {
                Box[1].position = Point(0, 0);
                Box[1].height = ImgParamsOri.height;
                Box[1].width = ImgParamsOri.width;
            }
            return;
        }
        axesPoints.clear();
        for (int j = 0; j < model_3D[1].corners.size(); j++) {
            axesPoints.push_back(model_3D[1].corners[j]);
        }

        cnt = 1, x_min = ImgParamsOri.width, y_min = ImgParamsOri.height, x_max = -1, y_max = -1;
        imagePoints.clear();
        Rot = Mat::zeros(3, 3, CV_32FC1);
        Rodrigues(Poses.opener_pose.rotation, Rot);
        Rot = camera_parameters.Rotation * Rot;
        rvec = Mat::zeros(3, 1, CV_32FC1);
        Rodrigues(Rot, rvec);
        projectPoints(axesPoints, rvec, camera_parameters.Rotation * Poses.opener_pose.translation + camera_parameters.Translation, camera_parameters.Intrinsic, camera_parameters.Distortion, imagePoints);

        for (int j = 0; j < imagePoints.size(); j++) {
            if (floor(imagePoints[j].x) < x_min) x_min = floor(imagePoints[j].x);
            if (floor(imagePoints[j].y) < y_min) y_min = floor(imagePoints[j].y);
            if (ceil(imagePoints[j].x) > x_max) x_max = ceil(imagePoints[j].x);
            if (ceil(imagePoints[j].y) > y_max) y_max = ceil(imagePoints[j].y);
        }
        Box[1].position = Point(max(0, x_min - BoxBorder), max(0, y_min - BoxBorder));
        Box[1].width = min(x_max - x_min + BoxBorder * 2, ImgParamsOri.width - Box[1].position.x - 1);
        Box[1].height = min(y_max - y_min + BoxBorder * 2, ImgParamsOri.height - Box[1].position.y - 1);
        Box[1].lostFrame = 0;
    }
    if (cameraType == USBCamera) {
        if (!Poses.endo_pose.recovery) {
            if (++Box[0].lostFrame > maxLostFrame) {
                Box[0].position = Point(0, 0);
                Box[0].height = ImgParamsEndo.height;
                Box[0].width = ImgParamsEndo.width;
            }
        }
        axesPoints.clear();
        for (int i = 0; i < model_3D[2].corners.size(); i++) {
            axesPoints.push_back(model_3D[2].corners[i]);
        }

        cnt = 1, x_min = ImgParamsOri.width, y_min = ImgParamsOri.height, x_max = -1, y_max = -1;
        imagePoints.clear();
        Rot = Mat::zeros(3, 3, CV_32FC1);
        Rodrigues(Poses.endo_pose.rotation, Rot);
        Rot = camera_parameters.Rotation * Rot;
        rvec = Mat::zeros(3, 1, CV_32FC1);
        Rodrigues(Rot, rvec);
        projectPoints(axesPoints, rvec, camera_parameters.Rotation * Poses.endo_pose.translation + camera_parameters.Translation, camera_parameters.Intrinsic, camera_parameters.Distortion, imagePoints);

        for (int i = 0; i < imagePoints.size(); i++) {
            if (floor(imagePoints[i].x) < x_min) x_min = floor(imagePoints[i].x);
            if (floor(imagePoints[i].y) < y_min) y_min = floor(imagePoints[i].y);
            if (ceil(imagePoints[i].x) > x_max) x_max = ceil(imagePoints[i].x);
            if (ceil(imagePoints[i].y) > y_max) y_max = ceil(imagePoints[i].y);
        }
        Box[0].position = Point(max(0, x_min - BoxBorder), max(0, y_min - BoxBorder));
        Box[0].width = min(x_max - x_min + BoxBorder * 2, ImgParamsOri.width - Box[0].position.x - 1);
        Box[0].height = min(y_max - y_min + BoxBorder * 2, ImgParamsOri.height - Box[0].position.y - 1);
        Box[0].lostFrame = 0;
    }
}
