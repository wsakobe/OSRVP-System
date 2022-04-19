#include "include/pose_estimation.h"
#include <fstream>
#include "omp.h"

using namespace cv;
using namespace std;

const Mat CamIntrinsicLeft = (Mat_<double>(3, 3) << 2185.86372107324, 0, 952.022350099373,
    0, 2186.58735329496, 563.875348654881,
    0, 0, 1);
const Mat DistCoeffLeft = (Mat_<double>(5, 1) << -0.170085848625626, 0.203029010848620, 0, 0, 0);

int number_of_corner_x_input, number_of_corner_y_input;

vector<cornerPreInfo> candidate_corners;
vector<cornerInformation> cornerPoints;
vector<corner_pos_with_ID> corner_pos_ID_left, corner_pos_ID_right, corner_pos_ID;
valueMatrix value_matrix[1025];
float model_3D[number_of_corner_x * number_of_corner_y][3];
int dot_matrix[number_of_corner_x][number_of_corner_y];
imageParams ImgParams;
PoseInformation Pose;

Mat image, image1, image_gray;
vector<Point3f> axesPoints;
vector<Point2f> imagePoints;

void initModel();
vector<corner_pos_with_ID> readMarker(Mat& image);
void plotModel(Mat& image, PoseInformation Pose);

int main(int argc, char* argv[]) {
    initModel();
    int start_time, last_time = 0, middle_time, middle_time1;
    //image = imread("F:\\OSRVP-System\\OSRVP-System\\OSRVP-System\\image.bmp");
    
    VideoCapture capture;
    image = capture.open("F:\\OSRVP-System\\OSRVP-System\\OSRVP-System\\Data\\left.avi");
    if (!capture.isOpened())
    {
        printf("can not open ...\n");
        return -1;
    }
    while (capture.read(image)) {
        start_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        //cout << 1000.0 / (start_time - last_time) << endl;

        //Rect roi(900, 400, 1000, 600);
        //image = image(roi).clone();
        
        corner_pos_ID_left = readMarker(image);

        middle_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        
        //cout << corner_pos_ID_left.size() << endl;
        if (corner_pos_ID_left.size() < 4) {
            cout << "Not enough corners!" << endl;
            imshow("image_pose_pnp", image);
            waitKey(1);
            continue;
        }

        PoseEstimation pE;
        Pose = pE.poseEstimationMono(corner_pos_ID_left, CamIntrinsicLeft, DistCoeffLeft, model_3D);
        
        plotModel(image, Pose);

        last_time = start_time;
    }

    destroyAllWindows();
    return 0;
}

vector<corner_pos_with_ID> readMarker(Mat& image) {
    cvtColor(image, image_gray, COLOR_BGR2GRAY);
    image_gray.convertTo(image_gray, CV_32FC1); image_gray *= 1. / 255;

    ImgParams.height = image_gray.rows;
    ImgParams.width = image_gray.cols;

    PreFilter pF;
    candidate_corners = pF.preFilter(image_gray, number_of_corner_x_input * number_of_corner_y_input);

    FinalElection fE;
    cornerPoints = fE.finalElection(image_gray, candidate_corners);

    ArrayOrganization arrayOrg;
    int* matrix_p = arrayOrg.delaunayTriangulation(image_gray, cornerPoints);

    IdentifyMarker identify;
    corner_pos_ID = identify.identifyMarker(image_gray, matrix_p, cornerPoints, value_matrix, dot_matrix);
    
    return corner_pos_ID;
}

void initModel() {
    string filePath = "F:\\OSRVP-System\\OSRVP-System\\OSRVP-System\\Data\\";
    string modelName = filePath + "Model3D.txt";
    string valueMatrixName = filePath + "valueMatrix.txt";
    string dotMarixName = filePath + "dotMatrix.txt";

    ifstream Files;
    Files.open(modelName);
    if (!Files.is_open())
    {
        cout << "Cannot load Model3D.txt£¡" << endl;
        return;
    }
    int i;
    while(!Files.eof()) {
        Files >> i;
        Files >> model_3D[i][0] >> model_3D[i][1] >> model_3D[i][2];
    }
    Files.close();

    Files.open(valueMatrixName);
    if (!Files.is_open())
    {
        cout << "Cannot load valueMatrix.txt£¡" << endl;
        return;
    }
    int value;
    while (!Files.eof()) {
        Files >> value;
        Files >> value_matrix[value].pos.x >> value_matrix[value].pos.y >> value_matrix[value].dir;
    }
    Files.close();

    Files.open(dotMarixName);
    if (!Files.is_open())
    {
        cout << "Cannot load dotMatrix.txt£¡" << endl;
        return;
    }
    Files >> number_of_corner_x_input >> number_of_corner_y_input;
    for (int i = 0; i < number_of_corner_x_input; i++)
        for (int j = 0; j < number_of_corner_y_input; j++)
            Files >> dot_matrix[i][j];
}

void plotModel(Mat& image, PoseInformation Pose) {
    axesPoints.clear();
    imagePoints.clear();
    for (int i = 66; i < 71; i++)
        axesPoints.push_back(Point3f(model_3D[i][0], model_3D[i][1], model_3D[i][2]));
    projectPoints(axesPoints, Pose.rotation, Pose.translation, CamIntrinsicLeft, DistCoeffLeft, imagePoints);
    for (int i = 0; i < axesPoints.size() - 1; i++) {
        line(image, imagePoints[i], imagePoints[i + 1], Scalar(0, 100, 200), 3);
        //circle(image, imagePoints[i], 2, Scalar(0, 0, 255), -1);
    }
    
    axesPoints.clear();
    imagePoints.clear();
    for (int i = 70; i < 134; i += 7)
        axesPoints.push_back(Point3f(model_3D[i][0], model_3D[i][1], model_3D[i][2]));
    projectPoints(axesPoints, Pose.rotation, Pose.translation, CamIntrinsicLeft, DistCoeffLeft, imagePoints);
    for (int i = 0; i < axesPoints.size() - 1; i++) {
        line(image, imagePoints[i], imagePoints[i + 1], Scalar(0, 100, 200), 3);
        //circle(image, imagePoints[i], 2, Scalar(0, 0, 255));
    }

    axesPoints.clear();
    imagePoints.clear();
    for (int i = 66; i < 130; i += 7)
        axesPoints.push_back(Point3f(model_3D[i][0], model_3D[i][1], model_3D[i][2]));
    projectPoints(axesPoints, Pose.rotation, Pose.translation, CamIntrinsicLeft, DistCoeffLeft, imagePoints);
    for (int i = 0; i < axesPoints.size() - 1; i++) {
        line(image, imagePoints[i], imagePoints[i + 1], Scalar(0, 100, 200), 3);
        //circle(image, imagePoints[i], 2, Scalar(0, 0, 255));
    }

    axesPoints.clear();
    imagePoints.clear();
    for (int i = 129; i < 134; i++)
        axesPoints.push_back(Point3f(model_3D[i][0], model_3D[i][1], model_3D[i][2]));
    projectPoints(axesPoints, Pose.rotation, Pose.translation, CamIntrinsicLeft, DistCoeffLeft, imagePoints);
    for (int i = 0; i < axesPoints.size() - 1; i++) {
        line(image, imagePoints[i], imagePoints[i + 1], Scalar(0, 100, 0), 2);
        //circle(image, imagePoints[i], 2, Scalar(0, 0, 255));
    }
    
    //»æÖÆÄ©¶ËÖ´ÐÐÆ÷Î»×Ë
    axesPoints.clear();
    for (int i = 0; i < Pose.tracking_points.size(); i++)
        axesPoints.push_back(Pose.tracking_points[i]);
    projectPoints(axesPoints, Pose.rotation, Pose.translation, CamIntrinsicLeft, DistCoeffLeft, imagePoints);
    circle(image, imagePoints[0], 4, Scalar(120, 120, 0));

    imshow("image_pose_pnp", image);
    waitKey(1);
}