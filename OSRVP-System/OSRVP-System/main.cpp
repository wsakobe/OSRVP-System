#include "include/pose_estimation.h"
#include <fstream>
#include <algorithm>
#include <process.h>
#include <conio.h>
#include "mvcameracontrol.h"

using namespace cv;
using namespace std;

const Mat CamIntrinsicLeft = (Mat_<double>(3, 3) << 2185.86372107324, 0, 952.022350099373,
    0, 2186.58735329496, 563.875348654881,
    0, 0, 1);
const Mat DistCoeffLeft = (Mat_<double>(5, 1) << -0.170085848625626, 0.203029010848620, 0, 0, 0);
int BoxBorder = 6;

//ReadMarker Preparation
int number_of_corner_x_input, number_of_corner_y_input;
vector<cornerPreInfo> candidate_corners;
vector<cornerInformation> cornerPoints;
vector<corner_pos_with_ID> corner_pos_ID_left, corner_pos_ID_right, corner_pos_ID;
valueMatrix value_matrix[1025];
float model_3D[number_of_corner_x * number_of_corner_y][3];
int dot_matrix[number_of_corner_x][number_of_corner_y];
imageParams ImgParams;
PoseInformation Pose;
DynamicROIBox Box;

//HikVision Camera Preparation
int nRet = MV_OK;
int nRet1 = MV_OK;
int nRet2 = MV_OK;
void* handle1 = NULL;
void* handle2 = NULL;
unsigned char* pData1;
unsigned char* pData2;
unsigned int g_nPayloadSize = 0;
MV_FRAME_OUT_INFO_EX* imageInfo;
Mat Convert2Mat(MV_FRAME_OUT_INFO_EX* pstImageInfo, unsigned char* pData);
MV_CC_DEVICE_INFO_LIST stDeviceList;
MV_FRAME_OUT_INFO_EX stImageInfo1 = { 0 }, stImageInfo2 = { 0 };

Mat image, image_crop, image_gray;
vector<Point3f> axesPoints;
vector<Point2f> imagePoints;

void initModel();
void initCamera();
vector<corner_pos_with_ID> readMarker(Mat& image);
void plotModel(Mat& image, PoseInformation Pose);
DynamicROIBox dynamicROI(PoseInformation Pose, vector<corner_pos_with_ID> corner_pos_ID);

int start_time, last_time = 0;

int main(int argc, char* argv[]) {
    initModel();  

    //工业相机实时视频流
    initCamera();
    while (1) {
        if (kbhit()) {
            char ch = getch();
            if (ch == 27) {
                nRet1 = MV_CC_CloseDevice(handle1);
                nRet2 = MV_CC_CloseDevice(handle2);
                if ((MV_OK != nRet1) || (MV_OK != nRet2))
                {
                    printf("Close Device fail! nRet [0x%x]\n", nRet1);
                    break;
                }

                // Destroy handle
                nRet1 = MV_CC_DestroyHandle(handle1);
                nRet2 = MV_CC_DestroyHandle(handle2);
                if ((MV_OK != nRet1) || (MV_OK != nRet2))
                {
                    printf("Destroy Handle fail! nRet [0x%x]\n", nRet1);
                    break;
                }
                printf("Device successfully closed.\n");
                break;
            }
            ch = 0;
        }
        nRet1 = MV_CC_GetOneFrameTimeout(handle1, pData1, g_nPayloadSize, &stImageInfo1, 100);
        nRet2 = MV_CC_GetOneFrameTimeout(handle2, pData2, g_nPayloadSize, &stImageInfo2, 100);
        if ((MV_OK == nRet1) && (MV_OK == nRet2)) {
            image = Convert2Mat(&stImageInfo1, pData1);
            image = Convert2Mat(&stImageInfo2, pData2);

            Rect roi(Box.position.x, Box.position.y, Box.width, Box.height);
            image_crop = image(roi);
            /*image_crop.copyTo(mask(roi));
            imshow("DynamicROI", mask);
            waitKey(1);*/

            corner_pos_ID_left = readMarker(image_crop);
            corner_pos_ID_left = readMarker(image_crop);

            if (corner_pos_ID_left.size() < 4) {
                cout << "Not enough corners!" << endl;
                imshow("image_pose_pnp", image);
                waitKey(1);
                if (++Box.lostFrame > 5) {
                    Box.position = Point(0, 0);
                    Box.height = image.rows;
                    Box.width = image.cols;
                }
                continue;
            }

            for (int i = 0; i < corner_pos_ID_left.size(); i++)
                corner_pos_ID_left[i].subpixel_pos += Point2f(Box.position);

            PoseEstimation pE;
            Pose = pE.poseEstimationMono(corner_pos_ID_left, CamIntrinsicLeft, DistCoeffLeft, model_3D);

            Box = dynamicROI(Pose, corner_pos_ID_left);

            plotModel(image, Pose);

            last_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            cout << last_time - start_time << endl;
        }
    }

    //视频流处理
    /*
    VideoCapture capture;
    image = capture.open("F:\\OSRVP-System\\OSRVP-System\\OSRVP-System\\Data\\left.avi");
    if (!capture.isOpened())
    {
        printf("can not open ...\n");
        return -1;
    }
    capture.read(image);
    Box.height = image.rows;
    Box.width = image.cols;
    while (capture.read(image)) {    
        start_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        
        //Mat mask = Mat::zeros(image.rows, image.cols, CV_32FC3);
        Rect roi(Box.position.x, Box.position.y, Box.width, Box.height);
        image_crop = image(roi);
        //image_crop.copyTo(mask(roi));
        //imshow("DynamicROI", mask);
        //waitKey(1);

        corner_pos_ID_left = readMarker(image_crop);
        corner_pos_ID_left = readMarker(image_crop);

        if (corner_pos_ID_left.size() < 4) {
            cout << "Not enough corners!" << endl;
            imshow("image_pose_pnp", image);
            waitKey(1);
            if (++Box.lostFrame > 5) {
                Box.position = Point(0, 0);
                Box.height = image.rows;
                Box.width = image.cols;
            }
            continue;
        }
        
        for (int i = 0; i < corner_pos_ID_left.size(); i++)
            corner_pos_ID_left[i].subpixel_pos += Point2f(Box.position);

        PoseEstimation pE;
        Pose = pE.poseEstimationMono(corner_pos_ID_left, CamIntrinsicLeft, DistCoeffLeft, model_3D);

        Box = dynamicROI(Pose, corner_pos_ID_left);

        plotModel(image, Pose);

        last_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        cout << last_time - start_time << endl;
    }
    */
    destroyAllWindows();
    return 0;
}

vector<corner_pos_with_ID> readMarker(Mat& image) {
    cvtColor(image, image_gray, COLOR_BGR2GRAY);
    image_gray.convertTo(image_gray, CV_32FC1); image_gray *= 1. / 255;
    corner_pos_ID.clear();

    ImgParams.height = image_gray.rows;
    ImgParams.width = image_gray.cols;

    PreFilter pF;
    candidate_corners = pF.preFilter(image_gray, number_of_corner_x_input * number_of_corner_y_input);

    FinalElection fE;
    cornerPoints = fE.finalElection(image_gray, candidate_corners);

    if (cornerPoints.size() > 4) {
        ArrayOrganization arrayOrg;
        int* matrix_p = arrayOrg.delaunayTriangulation(image_gray, cornerPoints);

        IdentifyMarker identify;
        corner_pos_ID = identify.identifyMarker(image_gray, matrix_p, cornerPoints, value_matrix, dot_matrix);
    }
    
    return corner_pos_ID;
}

DynamicROIBox dynamicROI(PoseInformation Pose, vector<corner_pos_with_ID> corner_pos_ID) {
    int cnt = 1, x_min = 10000, y_min = 10000, x_max = -1, y_max = -1;
    axesPoints.clear();
    imagePoints.clear();

    while (cnt < number_of_corner_x * number_of_corner_y) {
        if ((model_3D[cnt][0] - 0.0 > 1e-3) || (model_3D[cnt][1] - 0.0 > 1e-3) || (model_3D[cnt][2] - 0.0 > 1e-3))
            axesPoints.push_back(Point3f(model_3D[cnt][0], model_3D[cnt][1], model_3D[cnt][2]));
        cnt++;
    }
    projectPoints(axesPoints, Pose.rotation, Pose.translation, CamIntrinsicLeft, DistCoeffLeft, imagePoints);

    for (int i = 0; i < imagePoints.size(); i++) {
        if (floor(imagePoints[i].x) < x_min) x_min = floor(imagePoints[i].x);
        if (floor(imagePoints[i].y) < y_min) y_min = floor(imagePoints[i].y);
        if (ceil(imagePoints[i].x) > x_max) x_max = ceil(imagePoints[i].x);
        if (ceil(imagePoints[i].y) > y_max) y_max = ceil(imagePoints[i].y);
    }
    Box.position = Point(max(0, x_min - BoxBorder), max(0, y_min - BoxBorder));
    Box.width = x_max - x_min + BoxBorder * 2;
    Box.height  = y_max - y_min + BoxBorder * 2;
    Box.lostFrame = 0;

    return Box;
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
        cout << "Cannot load Model3D.txt！" << endl;
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
        cout << "Cannot load valueMatrix.txt！" << endl;
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
        cout << "Cannot load dotMatrix.txt！" << endl;
        return;
    }
    Files >> number_of_corner_x_input >> number_of_corner_y_input;
    for (int i = 0; i < number_of_corner_x_input; i++)
        for (int j = 0; j < number_of_corner_y_input; j++)
            Files >> dot_matrix[i][j];

    Box.position = Point(0, 0);
}

void initCamera() {
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

    nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (MV_OK != nRet)
    {
        printf("Enum Devices fail! nRet [0x%x]\n", nRet);
    }

    nRet1 = MV_CC_CreateHandle(&handle1, stDeviceList.pDeviceInfo[0]);
    nRet2 = MV_CC_CreateHandle(&handle2, stDeviceList.pDeviceInfo[1]);
    if ((MV_OK != nRet1) || (MV_OK != nRet2))
    {
        printf("Create Handle fail! nRet [0x%x]\n", nRet1);
    }

    nRet1 = MV_CC_OpenDevice(handle1);
    nRet2 = MV_CC_OpenDevice(handle2);
    if ((MV_OK != nRet1) || (MV_OK != nRet2))
    {
        printf("Open Device fail! nRet [0x%x]\n", nRet1);
    }

    nRet1 = MV_CC_SetEnumValue(handle1, "TriggerMode", 0);
    nRet2 = MV_CC_SetEnumValue(handle2, "TriggerMode", 0);
    if ((MV_OK != nRet1) || (MV_OK != nRet2))
    {
        printf("Set Enum Value fail! nRet [0x%x]\n", nRet1);
    }

    // Get payload size
    MVCC_INTVALUE stParam;
    memset(&stParam, 0, sizeof(MVCC_INTVALUE));
    nRet = MV_CC_GetIntValue(handle1, "PayloadSize", &stParam);
    g_nPayloadSize = stParam.nCurValue;

    nRet1 = MV_CC_StartGrabbing(handle1);
    nRet2 = MV_CC_StartGrabbing(handle2);
    if ((MV_OK != nRet1) || (MV_OK != nRet2))
    {
        printf("Start Grabbing fail! nRet [0x%x]\n", nRet1);
    }

    memset(&stImageInfo1, 0, sizeof(MV_FRAME_OUT_INFO_EX));
    memset(&stImageInfo2, 0, sizeof(MV_FRAME_OUT_INFO_EX));
    pData1 = (unsigned char*)malloc(sizeof(unsigned char) * (g_nPayloadSize));
    pData2 = (unsigned char*)malloc(sizeof(unsigned char) * (g_nPayloadSize));
}

void plotModel(Mat& image, PoseInformation Pose) {
    axesPoints.clear();
    imagePoints.clear();
    for (int i = 66; i < 71; i++)
        axesPoints.push_back(Point3f(model_3D[i][0], model_3D[i][1], model_3D[i][2]));
    for (int i = 70; i < 134; i += 7)
        axesPoints.push_back(Point3f(model_3D[i][0], model_3D[i][1], model_3D[i][2]));
    for (int i = 133; i >= 129; i--)
        axesPoints.push_back(Point3f(model_3D[i][0], model_3D[i][1], model_3D[i][2]));
    for (int i = 129; i >= 66; i -= 7)
        axesPoints.push_back(Point3f(model_3D[i][0], model_3D[i][1], model_3D[i][2]));
    
    projectPoints(axesPoints, Pose.rotation, Pose.translation, CamIntrinsicLeft, DistCoeffLeft, imagePoints);
    for (int i = 0; i < axesPoints.size() - 1; i++) {
        line(image, imagePoints[i], imagePoints[i + 1], Scalar(0, 100, 200), 3);
        //circle(image, imagePoints[i], 2, Scalar(0, 0, 255), -1);
    }
    
    //绘制末端执行器位姿
    axesPoints.clear();
    for (int i = 0; i < Pose.tracking_points.size(); i++)
        axesPoints.push_back(Pose.tracking_points[i]);
    projectPoints(axesPoints, Pose.rotation, Pose.translation, CamIntrinsicLeft, DistCoeffLeft, imagePoints);
    circle(image, imagePoints[0], 4, Scalar(120, 120, 0));

    imshow("image_pose", image);
    waitKey(1);
}

int RGB2BGR(unsigned char* pRgbData, unsigned int nWidth, unsigned int nHeight)
{
    if (NULL == pRgbData)
    {
        return MV_E_PARAMETER;
    }

    for (unsigned int j = 0; j < nHeight; j++)
    {
        for (unsigned int i = 0; i < nWidth; i++)
        {
            unsigned char red = pRgbData[j * (nWidth * 3) + i * 3];
            pRgbData[j * (nWidth * 3) + i * 3] = pRgbData[j * (nWidth * 3) + i * 3 + 2];
            pRgbData[j * (nWidth * 3) + i * 3 + 2] = red;
        }
    }

    return MV_OK;
}

// convert data stream in Mat format
cv::Mat Convert2Mat(MV_FRAME_OUT_INFO_EX* pstImageInfo, unsigned char* pData)
{
    cv::Mat srcImage;
    if (pstImageInfo->enPixelType == PixelType_Gvsp_Mono8)
    {
        srcImage = cv::Mat(pstImageInfo->nHeight, pstImageInfo->nWidth, CV_8UC1, pData);
    }
    else if (pstImageInfo->enPixelType == PixelType_Gvsp_RGB8_Packed)
    {
        srcImage = cv::Mat(pstImageInfo->nHeight, pstImageInfo->nWidth, CV_8UC3, pData);
    }

    cv::waitKey(20);

    return srcImage;
}