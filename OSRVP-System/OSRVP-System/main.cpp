#include "include/pose_estimation.h"
#include <algorithm>
#include <process.h>
#include <conio.h>
#include <thread>
#include "mvcameracontrol.h"

using namespace cv;
using namespace std;

//ReadMarker Preparation
int camera_number, tracking_number, number_of_corner_x_input, number_of_corner_y_input;
vector<CameraParams> camera_parameters;
CameraParams cam;
valueMatrix value_matrix[1025];
float model_3D[number_of_corner_x * number_of_corner_y][3];
int dot_matrix[number_of_corner_x][number_of_corner_y];

imageParams ImgParams, ImgParamsOri;
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

Mat image, image_crop, image_gray, image_firstcam;
vector<Point3f> axesPoints;
vector<Point2f> imagePoints;

void initModel();
bool initCamera();
vector<corner_pos_with_ID> readMarker(Mat& image);
void plotModel(Mat& image, PoseInformation Pose, vector<CameraParams> camera_parameters);
void dynamicROI(PoseInformation Pose, vector<CameraParams> camera_parameters);

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
        time_start = GetTickCount();
        //cout << "Time = " << 1000.0 / (time_start - time_end) << "FPS\n ";
        time_end = time_start;
        
        corner_pos_ID.clear();
        for (int i = 0; i < stDeviceList.nDeviceNum; i++) {
            nRet[i] = MV_CC_GetImageBuffer(handle[i], &stImageInfo[i], 1000);
            if (nRet[i] == MV_OK)
            {
                image = Convert2Mat(stImageInfo[i]);
                if (i == 0)  image_firstcam = image.clone();
                //imwrite("image.bmp", image);
                //Mat mask = Mat::zeros(image.rows, image.cols, CV_8UC1);
                Rect roi(Box[i].position.x, Box[i].position.y, Box[i].width, Box[i].height);
                image_crop = image(roi);
                //image_crop.copyTo(mask(roi));
                //imshow("DynamicROI", mask);
                //waitKey(1);
                
                corner_pos_ID.push_back(readMarker(image_crop));

                for (int j = 0; j < corner_pos_ID[i].size(); j++)
                    corner_pos_ID[i][j].subpixel_pos += Point2f(Box[i].position);
            }
            nRet[i] = MV_CC_FreeImageBuffer(handle[i], &stImageInfo[i]);
            if (nRet[i] != MV_OK)
            {
                printf("Free Image Buffer fail! nRet [0x%x]\n", nRet[i]);
            } 
        }
        
        PoseEstimation pE;
        Pose = pE.poseEstimation(corner_pos_ID, camera_parameters, model_3D, stDeviceList.nDeviceNum);
        
        if (!Pose.recovery) {
            //cout << "Fail to localize the model!" << endl;
            imshow("image_pose", image_firstcam);
            waitKey(1);
        }
        dynamicROI(Pose, camera_parameters);

        plotModel(image_firstcam, Pose, camera_parameters);
                
        if (g_bExit)
        {
            break;
        }
    }
    
}

int main(int argc, char* argv[]) {
    initModel();
    google::InitGoogleLogging(argv[0]);
    
    //工业相机实时视频流
    if (!initCamera()) return 0;
    
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
    
    //视频流处理
    /*
    //VideoCapture capture;
    //image = capture.open("F:\\OSRVP-System\\OSRVP-System\\OSRVP-System\\Data\\left.avi");
    image = imread("F:\\OSRVP-System\\OSRVP-System\\OSRVP-System\\image.bmp");

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
        Pose = pE.poseEstimation(corner_pos_ID, camera_parameters, model_3D, stDeviceList.nDeviceNum);

        if (!Pose.recovery) {
            //cout << "Fail to localize the model!" << endl;
            imshow("image_pose", image);
            waitKey(1);
        }
        dynamicROI(Pose, camera_parameters);
        //plotModel(image, Pose, camera_parameters);
    //}
    */
    destroyAllWindows();
    return 0;
}

vector<corner_pos_with_ID> readMarker(Mat& image) {
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
    
    if (cornerPoints.size() > 4) {
        ArrayOrganization arrayOrg;
        int* matrix_p = arrayOrg.delaunayTriangulation(image_gray, cornerPoints);
        
        IdentifyMarker identify;
        corner_pos = identify.identifyMarker(image_gray, matrix_p, cornerPoints, value_matrix, dot_matrix);
    }
    
    return corner_pos;
}

void dynamicROI(PoseInformation Pose, vector<CameraParams> camera_parameters) {
    if (!Pose.recovery){
        for (int num = 0; num < stDeviceList.nDeviceNum; num++) {
            if (++Box[num].lostFrame > maxLostFrame) {
                Box[num].position = Point(0, 0);
                Box[num].height = ImgParamsOri.height;
                Box[num].width = ImgParamsOri.width;
            }
        }            
        return;
    }
    axesPoints.clear();
    while (cnt < number_of_corner_x * number_of_corner_y) {
        if ((model_3D[cnt][0] - 0.0 > 1e-3) || (model_3D[cnt][1] - 0.0 > 1e-3) || (model_3D[cnt][2] - 0.0 > 1e-3))
            axesPoints.push_back(Point3f(model_3D[cnt][0], model_3D[cnt][1], model_3D[cnt][2]));
        cnt++;
    }
    
    for (int num = 0; num < stDeviceList.nDeviceNum; num++) {
        cnt = 1, x_min = ImgParamsOri.width, y_min = ImgParamsOri.height, x_max = -1, y_max = -1;
        imagePoints.clear();
        Mat Rot = Mat::zeros(3, 3, CV_32FC1);
        Rodrigues(Pose.rotation, Rot);
        Rot = camera_parameters[num].Rotation * Rot;
        Mat rvec = Mat::zeros(3, 1, CV_32FC1);
        Rodrigues(Rot, rvec);
        projectPoints(axesPoints, rvec, camera_parameters[num].Rotation * Pose.translation + camera_parameters[num].Translation, camera_parameters[num].Intrinsic, camera_parameters[num].Distortion, imagePoints);
        
        for (int i = 0; i < imagePoints.size(); i++) {
            if (floor(imagePoints[i].x) < x_min) x_min = floor(imagePoints[i].x);
            if (floor(imagePoints[i].y) < y_min) y_min = floor(imagePoints[i].y);
            if (ceil(imagePoints[i].x) > x_max) x_max = ceil(imagePoints[i].x);
            if (ceil(imagePoints[i].y) > y_max) y_max = ceil(imagePoints[i].y);
        }
        Box[num].position = Point(max(0, x_min - BoxBorder), max(0, y_min - BoxBorder));
        Box[num].width = min(x_max - x_min + BoxBorder * 2, ImgParamsOri.width - Box[num].position.x - 1);
        Box[num].height = min(y_max - y_min + BoxBorder * 2, ImgParamsOri.height - Box[num].position.y - 1);
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
    fs["imageWidth"] >> ImgParamsOri.width;
    fs["imageHeight"] >> ImgParamsOri.height;

    string cameraMatrix = "cameraMatrix";
    string distCoeffs = "distCoeffs";
    string Rotation = "Rotation";
    string Translation = "Translation";
    for (int i = 0; i < camera_number; i++) {
        memset((void*)&cam, 0x00, sizeof(cam));
        Box[i].height = ImgParamsOri.height;
        Box[i].width = ImgParamsOri.width;
        
        string cameraMatrixi = cameraMatrix + to_string(i + 1);
        string distCoeffsi = distCoeffs + to_string(i + 1);
        string Rotationi = Rotation + to_string(i + 1);
        string Translationi = Translation + to_string(i + 1);
        
        fs[cameraMatrixi] >> cam.Intrinsic;
        fs[distCoeffsi] >> cam.Distortion;
        fs[Rotationi] >> cam.Rotation;
        fs[Translationi] >> cam.Translation;

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
    cout << "Camera number: " << stDeviceList.nDeviceNum << endl;
    return true;
}

void plotModel(Mat& image, PoseInformation Pose, vector<CameraParams> camera_parameters) {
    if (!Pose.recovery) return;
    Mat imgMark(image.rows, image.cols, CV_32FC3);
    if (image.channels() != 1)
        imgMark = image.clone();
    else
        cvtColor(image, imgMark, COLOR_GRAY2RGB);
    
    axesPoints.clear();
    imagePoints.clear();
    for (int i = 37; i < 42; i++)
        axesPoints.push_back(Point3f(model_3D[i][0], model_3D[i][1], model_3D[i][2]));
    for (int i = 42; i < 63; i += 7)
        axesPoints.push_back(Point3f(model_3D[i][0], model_3D[i][1], model_3D[i][2]));
    for (int i = 63; i >= 58; i--)
        axesPoints.push_back(Point3f(model_3D[i][0], model_3D[i][1], model_3D[i][2]));
    for (int i = 58; i >= 37; i -= 7)
        axesPoints.push_back(Point3f(model_3D[i][0], model_3D[i][1], model_3D[i][2]));
    
    projectPoints(axesPoints, Pose.rotation, Pose.translation, camera_parameters[0].Intrinsic, camera_parameters[0].Distortion, imagePoints);
    for (int i = 1; i < axesPoints.size(); i++) {
        line(imgMark, imagePoints[i - 1], imagePoints[i], Scalar(0, 224, 158), 2);
        //circle(image, imagePoints[i], 2, Scalar(0, 0, 255), -1);
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

    return srcImage;
}