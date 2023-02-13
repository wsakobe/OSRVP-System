#include "../include/init.h"
#include "../include/config.h"

using namespace std;
using namespace cv;

void initData() {
    int camera_number, tracking_number;
    CameraParams cam;
    int model_num, point_num, value;
    int number_of_corner_x_input, number_of_corner_y_input;
    imageParams ImgParams, ImgParamsOri, ImgParamsEndo;
    valueMatrix value_matrix[1025];
    int dot_matrix[2 * number_of_corner_x][2 * number_of_corner_y];

    string filePath = "F:\\OSRVP-System\\OSRVP-System\\OSRVP-System\\Data\\";
    string markerType = "15x14";
    string modelName = filePath + "Model3D_suture.txt";
    string valueMatrixName = filePath + "valueMatrix_" + markerType + ".txt";
    string dotMarixName = filePath + "dotMatrix_" + markerType + ".txt";
    string cameraParametersName = filePath + "cameraParams.yml";

    ifstream Files;
    Files.open(modelName);
    if (!Files.is_open())
    {
        cout << "Cannot load Model3D.txt£¡" << endl;
        return;
    }
    
    Files >> model_num;
    model_3D.resize(model_num);
    for (int j = 0; j < model_num; j++) {
        Files >> point_num;
        model_3D[j].corners.resize(point_num);
        for (int k = 0; k < point_num; k++)
            Files >> model_3D[j].corners[k].x >> model_3D[j].corners[k].y >> model_3D[j].corners[k].z;
    }
    Files.close();

    Files.open(valueMatrixName);
    if (!Files.is_open())
    {
        cout << "Cannot load valueMatrix.txt£¡" << endl;
        return;
    }
    
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
            Files >> dot_matrix[j][i];

    FileStorage fs(cameraParametersName, FileStorage::READ);

    fs["cameraNumber"] >> camera_number;
    fs["imageWidth"] >> ImgParamsOri.width;
    fs["imageHeight"] >> ImgParamsOri.height;

    string cameraMatrix = "cameraMatrix";
    string distCoeffs = "distCoeffs";
    string Rotation = "Rotation";
    string Translation = "Translation";
    Box[0].resize(2);
    Box[1].resize(2);
    Box[2].resize(1);
    Box[0][0].height = ImgParamsOri.height;
    Box[0][0].width = ImgParamsOri.width;
    Box[0][1].height = ImgParamsOri.height;
    Box[0][1].width = ImgParamsOri.width;
    Box[1][0].height = ImgParamsOri.height;
    Box[1][0].width = ImgParamsOri.width;
    Box[1][1].height = ImgParamsOri.height;
    Box[1][1].width = ImgParamsOri.width;
    for (int i = 0; i < camera_number; i++) {
        memset((void*)&cam, 0x00, sizeof(cam));

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

    fs["imageWidthEndo"] >> ImgParamsEndo.width;
    fs["imageHeightEndo"] >> ImgParamsEndo.height;
    Box[2][0].height = ImgParamsOri.height;
    Box[2][0].width = ImgParamsOri.width;

    string cameraMatrixEndo = "cameraMatrixEndoscopy";
    string distCoeffsEndo = "distCoeffsEndoscopy";
    memset((void*)&cam, 0x00, sizeof(cam));
    fs[cameraMatrixEndo] >> cam.Intrinsic;
    fs[distCoeffsEndo] >> cam.Distortion;
    cam.Rotation = Mat::zeros(3, 3, CV_32FC1);
    cam.Translation = Mat::zeros(3, 1, CV_32FC1);
    endo_parameter.push_back(cam);

    fs.release();    	//close the file opened
}
