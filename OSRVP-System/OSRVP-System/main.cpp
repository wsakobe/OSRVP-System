#include "include/pose_estimation.h"
#include <fstream>
#include <algorithm>
#include <process.h>
#include <conio.h>
#include <thread>
#include "mvcameracontrol.h"

using namespace cv;
using namespace std;

//ReadMarker Preparation
int camera_number, number_of_corner_x_input, number_of_corner_y_input;
vector<CameraParams> camera_parameters;
CameraParams cam;
valueMatrix value_matrix[1025];
float model_3D[number_of_corner_x * number_of_corner_y][3];
int dot_matrix[number_of_corner_x][number_of_corner_y];

imageParams ImgParams;
vector<cornerPreInfo> candidate_corners;
vector<cornerInformation> cornerPoints;
vector<corner_pos_with_ID> corner_pos;
vector<vector<corner_pos_with_ID>> corner_pos_ID;

PoseInformation Pose;
DynamicROIBox Box[5];
int cnt = 1, x_min = 10000, y_min = 10000, x_max = -1, y_max = -1;
int BoxBorder = 10;
bool g_bExit = false;

//HikVision Camera Preparation
int nRet[5] = { MV_OK };
void* handle[5] = { NULL };
unsigned char* pData[5];
unsigned int g_nPayloadSize = 0;
MV_FRAME_OUT_INFO_EX* imageInfo;
Mat Convert2Mat(MV_FRAME_OUT& pstImageInfo);
MV_CC_DEVICE_INFO_LIST stDeviceList;
MV_FRAME_OUT stImageInfo[5] = { { 0 } };

Mat image, image_crop, image_gray;
vector<Point3f> axesPoints;
vector<Point2f> imagePoints;

void initModel();
bool initCamera();
vector<corner_pos_with_ID> readMarker(Mat& image);
void plotModel(Mat& image, PoseInformation Pose, vector<CameraParams> camera_parameters);
void dynamicROI(PoseInformation Pose, vector<CameraParams> camera_parameters);

int start_time, last_time = 0;

void WaitForKeyPress(void)
{
    while (!_kbhit())
    {
        waitKey(10);
    }
    if (_getch() == 27) g_bExit = true;
}

void WorkThread(void* handle[5]) {
    while (1) {
        for (int i = 0; i < stDeviceList.nDeviceNum; i++) {
            nRet[i] = MV_CC_GetImageBuffer(handle[i], &stImageInfo[i], 1000);
            if (nRet[i] == MV_OK)
            {
                image = Convert2Mat(stImageInfo[i]);
                imshow("image", image);
                waitKey(1);
                /*
                Rect roi(Box[i].position.x, Box[i].position.y, Box[i].width, Box[i].height);
                image_crop = image(roi);
                //image_crop.copyTo(mask(roi));
                //imshow("DynamicROI", mask);
                //waitKey(1);

                corner_pos_ID[i] = readMarker(image_crop);

                if (corner_pos_ID.size() < 4) {
                    cout << "Not enough corners!" << endl;
                    imshow("image_pose", image);
                    waitKey(1);
                    if (++Box[i].lostFrame > 5) {
                        Box[i].position = Point(0, 0);
                        Box[i].height = image.rows;
                        Box[i].width = image.cols;
                    }
                    continue;
                }

                for (int j = 0; j < corner_pos_ID[i].size(); j++)
                    corner_pos_ID[i][j].subpixel_pos += Point2f(Box[i].position);*/
            }
            nRet[i] = MV_CC_FreeImageBuffer(handle[i], &stImageInfo[i]);
            if (nRet != MV_OK)
            {
                printf("Free Image Buffer fail! nRet [0x%x]\n", nRet);
            }
            start_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            cout << start_time - last_time << endl;
            last_time = start_time;
        }
        //PoseEstimation pE;
        //Pose = pE.poseEstimation(corner_pos_ID, camera_parameters, model_3D, stDeviceList.nDeviceNum);

        //dynamicROI(Pose, camera_parameters);

        //plotModel(image, Pose, camera_parameters);

        if (g_bExit)
        {
            break;
        }
    }
    
}

int main(int argc, char* argv[]) {
    initModel();

    //工业相机实时视频流
    if (!initCamera()) return 0;
    
    std::thread thread_1(WorkThread, handle);
    thread_1.detach();

#pragma region AfterThread

     printf("Press [ Esc ] to stop grabbing.\n");
     WaitForKeyPress();
        
    for (int i = 0; i < stDeviceList.nDeviceNum; i++) {
        nRet[i] = MV_CC_CloseDevice(handle[i]);
        if (MV_OK != nRet[i])
        {
            printf("Close Device fail! nRet [0x%x]\n", nRet[i]);
            break;
        }
    }
    // Destroy handle
    for (int i = 0; i < stDeviceList.nDeviceNum; i++) {
        nRet[i] = MV_CC_DestroyHandle(handle[i]);
        if (MV_OK != nRet[i])
        {
            printf("Destroy Handle fail! nRet [0x%x]\n", nRet[i]);
            break;
        }
    }
    printf("Device successfully closed.\n");
        
#pragma endregion       

    
    
    //视频流处理
    /*
    VideoCapture capture;
    image = capture.open("F:\\OSRVP-System\\OSRVP-System\\OSRVP-System\\Data\\left.avi");
    //image = imread("F:\\OSRVP-System\\OSRVP-System\\OSRVP-System\\image.bmp");
    if (!capture.isOpened())
    {
        printf("can not open ...\n");
        return -1;
    }
    capture.read(image);
    stDeviceList.nDeviceNum = 1;
    while (capture.read(image)) {
        start_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        cout << 1000.0 / (start_time - last_time) << endl;
        last_time = start_time;
        
        //Mat mask = Mat::zeros(image.rows, image.cols, CV_32FC3);
        Rect roi(Box[0].position.x, Box[0].position.y, Box[0].width, Box[0].height);
        image_crop = image(roi);
        //image_crop.copyTo(mask(roi));
        //imshow("DynamicROI", mask);
        //waitKey(1);

        corner_pos_ID.push_back(readMarker(image_crop));
        //corner_pos_ID.push_back(readMarker(image_crop));

        cout << corner_pos_ID[0].size() << endl;
        if (corner_pos_ID[0].size() < 4) {
            cout << "Not enough corners!" << endl;
            imshow("image_pose", image);
            waitKey(1);
            if (++Box[0].lostFrame > 5) {
                Box[0].position = Point(0, 0);
                Box[0].height = image.rows;
                Box[0].width = image.cols;
            }
            continue;
        }

        for (int i = 0; i < corner_pos_ID[0].size(); i++)  corner_pos_ID[0][i].subpixel_pos += Point2f(Box[0].position);
        PoseEstimation pE;
        Pose = pE.poseEstimation(corner_pos_ID, camera_parameters, model_3D, stDeviceList.nDeviceNum);

        dynamicROI(Pose, camera_parameters);

        plotModel(image, Pose, camera_parameters);
    }
    */
    destroyAllWindows();
    return 0;
}

vector<corner_pos_with_ID> readMarker(Mat& image) {
    //cvtColor(image, image_gray, COLOR_BGR2GRAY);
    image_gray = image;
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
        corner_pos = identify.identifyMarker(image_gray, matrix_p, cornerPoints, value_matrix, dot_matrix);
    }
    
    return corner_pos;
}

void dynamicROI(PoseInformation Pose, vector<CameraParams> camera_parameters) {
    axesPoints.clear();
    while (cnt < number_of_corner_x * number_of_corner_y) {
        if ((model_3D[cnt][0] - 0.0 > 1e-3) || (model_3D[cnt][1] - 0.0 > 1e-3) || (model_3D[cnt][2] - 0.0 > 1e-3))
            axesPoints.push_back(Point3f(model_3D[cnt][0], model_3D[cnt][1], model_3D[cnt][2]));
        cnt++;
    }
    
    for (int num = 0; num < stDeviceList.nDeviceNum; num++) {
        cnt = 1, x_min = 10000, y_min = 10000, x_max = -1, y_max = -1;
        imagePoints.clear();

        projectPoints(axesPoints, camera_parameters[num].Rotation * Pose.rotation, camera_parameters[num].Rotation * Pose.translation + camera_parameters[num].Translation, camera_parameters[num].Intrinsic, camera_parameters[num].Distortion, imagePoints);

        for (int i = 0; i < imagePoints.size(); i++) {
            if (floor(imagePoints[i].x) < x_min) x_min = floor(imagePoints[i].x);
            if (floor(imagePoints[i].y) < y_min) y_min = floor(imagePoints[i].y);
            if (ceil(imagePoints[i].x) > x_max) x_max = ceil(imagePoints[i].x);
            if (ceil(imagePoints[i].y) > y_max) y_max = ceil(imagePoints[i].y);
        }
        Box[num].position = Point(max(0, x_min - BoxBorder), max(0, y_min - BoxBorder));
        Box[num].width = x_max - x_min + BoxBorder * 2;
        Box[num].height  = y_max - y_min + BoxBorder * 2;
        Box[num].lostFrame = 0;
    }
}

void initModel() {
    string filePath = "F:\\OSRVP-System\\OSRVP-System\\OSRVP-System\\Data\\";
    string modelName = filePath + "Model3D.txt";
    string valueMatrixName = filePath + "valueMatrix.txt";
    string dotMarixName = filePath + "dotMatrix.txt";
    string cameraParametersName = filePath + "cameraParams.yml";

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

    FileStorage fs(cameraParametersName, FileStorage::READ);

    fs["cameraNumber"] >> camera_number;
    fs["imageWidth"] >> ImgParams.width;
    fs["imageHeight"] >> ImgParams.height;

    for (int i = 0; i < camera_number; i++) {
        Box[i].height = ImgParams.height;
        Box[i].width = ImgParams.width;
        fs["cameraMatrix"] >> cam.Intrinsic;
        fs["distCoeffs"] >> cam.Distortion;
        fs["Rotation"] >> cam.Rotation;
        fs["Translation"] >> cam.Translation;
        camera_parameters.push_back(cam);
    }

    fs.release();    	//close the file opened
}

bool initCamera() {
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

    nRet[0] = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (MV_OK != nRet[0])
    {
        printf("Enum Devices fail! nRet [0x%x]\n", nRet[0]);
        return false;
    }

    if (stDeviceList.nDeviceNum == 0) {
        printf("There is no device available.\n");
        return false;
    }

    for (int i = 0; i < stDeviceList.nDeviceNum; i++) {
        nRet[i] = MV_CC_CreateHandle(&handle[i], stDeviceList.pDeviceInfo[i]);
        if (MV_OK != nRet[i])
        {
            printf("Create Handle fail! nRet [0x%x]\n", nRet[i]);
            return false;
        }
    }

    for (int i = 0; i < stDeviceList.nDeviceNum; i++) {
        nRet[i] = MV_CC_OpenDevice(handle[i]);
        if (MV_OK != nRet[i])
        {
            printf("Open Device fail! nRet [0x%x]\n", nRet[i]);
            return false;
        }
    }      

    for (int i = 0; i < stDeviceList.nDeviceNum; i++) {
        nRet[i] = MV_CC_SetEnumValue(handle[i], "TriggerMode", 0);
        if (MV_OK != nRet[i])
        {
            printf("Set Enum Value fail! nRet [0x%x]\n", nRet[i]);
            return false;
        }
    }

    // Get payload size
    MVCC_INTVALUE stParam;
    memset(&stParam, 0, sizeof(MVCC_INTVALUE));
    nRet[0] = MV_CC_GetIntValue(handle[0], "PayloadSize", &stParam);
    g_nPayloadSize = stParam.nCurValue;

    for (int i = 0; i < stDeviceList.nDeviceNum; i++) {
        nRet[i] = MV_CC_StartGrabbing(handle[i]);
        if (MV_OK != nRet[i])
        {
            printf("Start Grabbing fail! nRet [0x%x]\n", nRet[i]);
            return false;
        }
    }
    for (int i = 0; i < stDeviceList.nDeviceNum; i++) {
        memset(&stImageInfo[i], 0, sizeof(MV_FRAME_OUT_INFO_EX));
        pData[i] = (unsigned char*)malloc(sizeof(unsigned char) * (g_nPayloadSize));
    }

    return true;
}

void plotModel(Mat& image, PoseInformation Pose, vector<CameraParams> camera_parameters) {
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
    
    projectPoints(axesPoints, Pose.rotation, Pose.translation, camera_parameters[0].Intrinsic, camera_parameters[0].Distortion, imagePoints);
    for (int i = 0; i < axesPoints.size() - 1; i++) {
        line(image, imagePoints[i], imagePoints[i + 1], Scalar(0, 100, 200), 3);
        //circle(image, imagePoints[i], 2, Scalar(0, 0, 255), -1);
    }
    
    //绘制末端执行器位姿
    axesPoints.clear();
    for (int i = 0; i < Pose.tracking_points.size(); i++)
        axesPoints.push_back(Pose.tracking_points[i]);
    projectPoints(axesPoints, Pose.rotation, Pose.translation, camera_parameters[0].Intrinsic, camera_parameters[0].Distortion, imagePoints);
    circle(image, imagePoints[0], 4, Scalar(120, 120, 0));
    cout << imagePoints[0] << endl;
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

cv::Mat Convert2Mat(MV_FRAME_OUT& pstImage)   // convert data stream in Mat format
{
    cv::Mat srcImage;
    if (pstImage.stFrameInfo.enPixelType == PixelType_Gvsp_Mono8)
    {
        srcImage = cv::Mat(pstImage.stFrameInfo.nHeight, pstImage.stFrameInfo.nWidth, CV_8UC1, pstImage.pBufAddr);
    }
    else if (pstImage.stFrameInfo.enPixelType == PixelType_Gvsp_RGB8_Packed)
    {
        srcImage = cv::Mat(pstImage.stFrameInfo.nHeight, pstImage.stFrameInfo.nWidth, CV_8UC3, pstImage.pBufAddr);
    }

    waitKey(1);

    return srcImage;
}