#include "include/init.h"
#include "include/read_marker.h"
#include "include/pose_estimation.h"

using namespace cv;
using namespace std;

//HikVision Camera Preparation
int nRet[5] = { MV_OK };
void* handle[5] = { NULL };
unsigned char* pData[5];
unsigned int g_nPayloadSize = 0;
MV_FRAME_OUT_INFO_EX* imageInfo;
Mat Convert2Mat(MV_FRAME_OUT& pstImageInfo);
MV_CC_DEVICE_INFO_LIST stDeviceList;
MV_FRAME_OUT stImageInfo[5] = { { 0 } };
VideoCapture cap(0);

ReadMarker rm;
vector<cornerMarkerInfo> corners_now;
vector<Point3f> axesPoints;
vector<Point2f> imagePoints;
vector<Mat> image_record(3);
Mat image, image_crop;
vector<cornerMarkerInfo> corners_all;
vector<vector<DynamicROIBox>> Box(3);
vector<CameraParams> camera_parameters, endo_parameter;
vector<ModelInfo> model_3D;
FinalPoseInformation final_pose, Poses;

bool g_bExit = false;

bool initCamera();
void plotModel(vector<Mat>& image, FinalPoseInformation Pose, vector<CameraParams> camera_parameters, vector<CameraParams> endo_parameters);
void readCamera(bool camera_type);
//void readFile();

int time_start, time_end = 0;

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
        time_start = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        //cout << "Time = " << 1000.0 / (time_start - time_end) << "FPS\n ";
        time_end = time_start;
        
        corners_all.clear();
        corners_all.resize(2);
        for (int i = 0; i < stDeviceList.nDeviceNum; i++) {
            nRet[i] = MV_CC_GetImageBuffer(handle[i], &stImageInfo[i], 50);
            if (nRet[i] == MV_OK)
            {
                image = Convert2Mat(stImageInfo[i]);
                image_record[i] = image.clone();
                corners_now.clear();
                for (int j = 0; j < Box[i].size(); j++) {
                    Rect roi(Box[i][j].position.x, Box[i][j].position.y, Box[i][j].width, Box[i][j].height);
                    image_crop = image(roi);

                    //image_crop.copyTo(mask(roi));
                    //imshow("DynamicROI", mask);
                    //waitKey(1);

                    corners_now.push_back(rm.readMarker(image_crop, HikingCamera, Box[i][j]));
                }              
                for (int j = 0; j < corners_now.size(); j++) {
                    for (int k = 0; k < corners_now[j].robot_marker.size(); k++) {
                        corners_all[i].robot_marker.push_back(corners_now[j].robot_marker[k]);
                    }
                    for (int k = 0; k < corners_now[j].opener_marker.size(); k++) {
                        corners_all[i].opener_marker.push_back(corners_now[j].opener_marker[k]);
                    }
                }
            }

            nRet[i] = MV_CC_FreeImageBuffer(handle[i], &stImageInfo[i]);
            if (nRet[i] != MV_OK)
            {
                printf("Free Image Buffer fail! nRet [0x%x]\n", nRet[i]);
            } 
        }
        
        PoseEstimation pE;
        Poses = pE.poseEstimation(corners_all, camera_parameters, model_3D, stDeviceList.nDeviceNum, HikingCamera);
        
        if (!Poses.robot_pose.recovery) {
            cout << "Fail to localize the robot" << endl;
        }
        else {
            final_pose.robot_pose = Poses.robot_pose;
        }
        if (!Poses.opener_pose.recovery) {
            cout << "Fail to localize the opener" << endl;
        }
        else {
            final_pose.opener_pose = Poses.opener_pose;
        }

        cap >> image;
        image_record[2] = image.clone();
        corners_now.clear();
        Rect roi(Box[2][0].position.x, Box[2][0].position.y, Box[2][0].width, Box[2][0].height);
        image_crop = image(roi);
        corners_now.push_back(rm.readMarker(image_crop, USBCamera, Box[2][0]));

        Poses = pE.poseEstimation(corners_now, endo_parameter, model_3D, 1, USBCamera);
        
        if (!Poses.endo_pose.recovery) {
            cout << "Fail to localize the endoscope" << endl;
        }
        else {
            final_pose.endo_pose = Poses.endo_pose;
        }

        rm.dynamicROI(final_pose, Box[0], camera_parameters[0], model_3D, HikingCamera);
        rm.dynamicROI(final_pose, Box[1], camera_parameters[1], model_3D, HikingCamera);
        rm.dynamicROI(final_pose, Box[2], endo_parameter[0], model_3D, USBCamera);
        plotModel(image_record, final_pose, camera_parameters, endo_parameter);
        waitKey(100);
        if (g_bExit)
        {
            break;
        }
    }
    
}

int main(int argc, char* argv[]) {
    initData();
    google::InitGoogleLogging(argv[0]);
    
    readCamera(USBCamera);
    //readFile();
    
    destroyAllWindows();
    return 0;
}

void readCamera(bool camera_type) {
    //工业相机+内窥镜实时视频流获取准备
    if (!initCamera()) return;

    //视频流获取
    do {
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
    } while (0);

#pragma endregion
}

/*
void readFile() {
    VideoCapture capture;
    //image = capture.open("F:\\OSRVP-System\\OSRVP-System\\OSRVP-System\\Data\\left.avi");
    image = imread("F:\\OSRVP-System\\OSRVP-System\\OSRVP-System\\Data\\grap\\1.bmp");

    //capture.read(image);
    stDeviceList.nDeviceNum = 1;
    //while (capture.read(image)) {
        //Mat mask = Mat::zeros(image.rows, image.cols, CV_32FC3);
        Rect roi(Box[0].position.x, Box[0].position.y, Box[0].width, Box[0].height);
        image_crop = image(roi);
        //image_crop.copyTo(mask(roi));
        //imshow("DynamicROI", mask);
        //waitKey(1);

        corner_pos_ID.push_back(readMarker(image_crop));

        for (int i = 0; i < corner_pos_ID[0].size(); i++)  corner_pos_ID[0][i].subpixel_pos += Point2f(Box[0].position);
        PoseEstimation pE;
        Pose = pE.poseEstimation(corner_pos_ID, camera_parameters, model_3D, stDeviceList.nDeviceNum, HikingCamera);

        if (!Pose.recovery) {
            cout << "Fail to localize the model!" << endl;
            imshow("image_pose", image);
            waitKey(1);
        }
        dynamicROI(Pose, camera_parameters);
        plotModel(image, Pose, camera_parameters);
    //}
}*/

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

    if (stDeviceList.nDeviceNum == 1) {
        printf("There is one HikCamera offline. Please check.\n");
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

    cap.set(CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(CAP_PROP_FRAME_HEIGHT, 720);
    cap.set(CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));

    if (!cap.isOpened())
    {
        cout << "couldn't open capture." << endl;
        return false;
    }

    return true;
}

void plotModel(vector<Mat>& image, FinalPoseInformation Pose, vector<CameraParams> camera_parameters, vector<CameraParams> endo_parameters) {
    imshow("Camera 1 view", image[0]);
    waitKey(1);
    imshow("Camera 2 view", image[1]);
    waitKey(1);
    imshow("Endoscope view", image[2]);
    waitKey(1);
    /*
    if (!Pose.recovery) return;
    Mat imgMark(image.rows, image.cols, CV_32FC3);
    if (image.channels() != 1)
        imgMark = image.clone();
    else
        cvtColor(image, imgMark, COLOR_GRAY2RGB);
    
    axesPoints.clear();
    imagePoints.clear();
    for (int i = 1; i < 8; i++)
        axesPoints.push_back(Point3f(model_3D[i][0], model_3D[i][1], model_3D[i][2]));
    for (int i = 7; i < 134; i += 7)
        axesPoints.push_back(Point3f(model_3D[i][0], model_3D[i][1], model_3D[i][2]));
    for (int i = 133; i >= 127; i--)
        axesPoints.push_back(Point3f(model_3D[i][0], model_3D[i][1], model_3D[i][2]));
    for (int i = 127; i >= 1; i -= 7)
        axesPoints.push_back(Point3f(model_3D[i][0], model_3D[i][1], model_3D[i][2]));
    
    projectPoints(axesPoints, Pose.rotation, Pose.translation, camera_parameters[0].Intrinsic, camera_parameters[0].Distortion, imagePoints);
    for (int i = 1; i < axesPoints.size(); i++) {
        //line(imgMark, imagePoints[i - 1], imagePoints[i], Scalar(0, 224, 158), 2);
        circle(imgMark, imagePoints[i - 1], 2, Scalar(0, 120, 220), -1);
    }
    
    axesPoints.clear();
    imagePoints.clear();
    for (int i = 1; i < 7; i++)
        axesPoints.push_back(Point3f(model_3D[i][0], model_3D[i][1], model_3D[i][2]));
    for (int i = 7; i < 35; i += 7)
        axesPoints.push_back(Point3f(model_3D[i][0], model_3D[i][1], model_3D[i][2]));
    for (int i = 35; i >= 29; i--)
        axesPoints.push_back(Point3f(model_3D[i][0], model_3D[i][1], model_3D[i][2]));
    for (int i = 29; i >= 1; i -= 7)
        axesPoints.push_back(Point3f(model_3D[i][0], model_3D[i][1], model_3D[i][2]));

    projectPoints(axesPoints, Pose.rotation, Pose.translation, camera_parameters[0].Intrinsic, camera_parameters[0].Distortion, imagePoints);
    for (int i = 1; i < axesPoints.size(); i++) {
        line(imgMark, imagePoints[i - 1], imagePoints[i], Scalar(0, 224, 158), 2);
        //circle(image, imagePoints[i], 2, Scalar(0, 0, 255), -1);
    }
    
    axesPoints.clear();
    imagePoints.clear();
    for (int i = 101; i < 105; i++)
        axesPoints.push_back(Point3f(model_3D[i][0], model_3D[i][1], model_3D[i][2]));
    for (int i = 105; i < 133; i += 7)
        axesPoints.push_back(Point3f(model_3D[i][0], model_3D[i][1], model_3D[i][2]));
    for (int i = 133; i >= 127; i--)
        axesPoints.push_back(Point3f(model_3D[i][0], model_3D[i][1], model_3D[i][2]));
    for (int i = 127; i >= 106; i -= 7)
        axesPoints.push_back(Point3f(model_3D[i][0], model_3D[i][1], model_3D[i][2]));

    projectPoints(axesPoints, Pose.rotation, Pose.translation, camera_parameters[0].Intrinsic, camera_parameters[0].Distortion, imagePoints);
    for (int i = 1; i < axesPoints.size(); i++) {
        line(imgMark, imagePoints[i - 1], imagePoints[i], Scalar(0, 224, 158), 2);
        //circle(image, imagePoints[i], 2, Scalar(0, 0, 255), -1);
    }
    
    axesPoints.clear();
    imagePoints.clear();
    for (int i = 64; i < 70; i++)
        axesPoints.push_back(Point3f(model_3D[i][0], model_3D[i][1], model_3D[i][2]));
    for (int i = 70; i < 98; i += 7)
        axesPoints.push_back(Point3f(model_3D[i][0], model_3D[i][1], model_3D[i][2]));
    for (int i = 98; i >= 92; i--)
        axesPoints.push_back(Point3f(model_3D[i][0], model_3D[i][1], model_3D[i][2]));
    for (int i = 92; i >= 64; i -= 7)
        axesPoints.push_back(Point3f(model_3D[i][0], model_3D[i][1], model_3D[i][2]));

    projectPoints(axesPoints, Pose.rotation, Pose.translation, camera_parameters[0].Intrinsic, camera_parameters[0].Distortion, imagePoints);
    for (int i = 1; i < axesPoints.size(); i++) {
        line(imgMark, imagePoints[i - 1], imagePoints[i], Scalar(0, 224, 158), 2);
        //circle(image, imagePoints[i], 2, Scalar(0, 0, 255), -1);
    }
    
    //绘制末端执行器位姿
    axesPoints.clear();
    for (int i = 0; i < Pose.tracking_points.size(); i++)
        axesPoints.push_back(Pose.tracking_points[i]);
    projectPoints(axesPoints, Pose.rotation, Pose.translation, camera_parameters[0].Intrinsic, camera_parameters[0].Distortion, imagePoints);
    circle(imgMark, imagePoints[0], 5, Scalar(250, 120, 0), -1);
        
    imshow("image_pose", imgMark);
    waitKey(1);*/
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

    return srcImage;
}