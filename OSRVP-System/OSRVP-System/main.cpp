#include "include/identify_marker.h"
#include <fstream>

using namespace cv;
using namespace std;

const Mat CamIntrinsicLeft = (Mat_<double>(3, 3) << 1.0, 0, 0,
                                                    0, 1.0, 0,
                                                    0, 0, 1);
const Mat DistCoeffLeft = (Mat_<double>(1, 5) << 0, 0, 0, 0, 0);

const int number_of_corner_x = 20;
const int number_of_corner_y = 20;

int number_of_corner_x_input, number_of_corner_y_input;

vector<cornerPreInfo> candidate_corners;
vector<cornerInformation> cornerPoints;
vector<corner_pos_with_ID> corner_pos_ID;
static valueMatrix value_matrix[1025];
static int model_3D[number_of_corner_x * number_of_corner_y][3], dot_matrix[number_of_corner_x][number_of_corner_y];
imageParams ImgParams;

Mat image, image_gray;

void initModel();

int main(int argc, char* argv[]) {
   // double start_time, end_time;
    initModel();
    for (int i = 0; i < 1; i++) {
        //start_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        image = imread(".\\Data\\1.jpg");
        
        cvtColor(image, image_gray, COLOR_BGR2GRAY);
        image_gray.convertTo(image_gray, CV_32FC1); image_gray *= 1./255;
                
        ImgParams.height = image.rows;
        ImgParams.width = image.cols;

        PreFilter pF;
        candidate_corners = pF.preFilter(image_gray, 2 * number_of_corner_x * number_of_corner_y);

        FinalElection fE;
        cornerPoints = fE.finalElection(image_gray, candidate_corners);

        ArrayOrganization arrayOrg;
        int *matrix_p = arrayOrg.delaunayTriangulation(image_gray, cornerPoints);

        IdentifyMarker identify;
        corner_pos_ID = identify.identifyMarker(image_gray, matrix_p, cornerPoints, value_matrix, dot_matrix);

        //end_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        //cout << "FInal-election:" << (end_time - start_time) << endl;
    }

    return 0;
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
    int fileLength = 0;
    Files >> fileLength;
    for (int i = 0; i < fileLength; i++) {
        Files >> model_3D[i][0] >> model_3D[i][1] >> model_3D[i][2];
    }

    Files.open(valueMatrixName);
    if (!Files.is_open())
    {
        cout << "Cannot load valueMatrix.txt£¡" << endl;
        return;
    }
    fileLength = 0;
    int value;
    Files >> fileLength;
    for (int i = 0; i < fileLength; i++) {
        Files >> value;
        Files >> value_matrix[value].pos.x >> value_matrix[value].pos.y >> value_matrix[value].dir;
    }

    Files.open(dotMarixName);
    Files >> number_of_corner_x_input >> number_of_corner_y_input;
    if (!Files.is_open())
    {
        cout << "Cannot load dotMatrix.txt£¡" << endl;
        return;
    }
    for (int i = 0; i < number_of_corner_x; i++)
        for (int j = 0; j < number_of_corner_y; j++)
            Files >> dot_matrix[i][j];
}