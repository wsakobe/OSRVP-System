#include "include/identify_marker.h"

using namespace cv;
using namespace std;

const Mat CamIntrinsicLeft = (Mat_<double>(3, 3) << 1.0, 0, 0,
                                                    0, 1.0, 0,
                                                    0, 0, 1);
const Mat DistCoeffLeft = (Mat_<double>(1, 5) << 0, 0, 0, 0, 0);

const int number_of_corner_x = 10;
const int number_of_corner_y = 10;

vector<cornerPreInfo> candidate_corners;
vector<cornerInformation> cornerPoints;

int model_3D[number_of_corner_x * number_of_corner_y][3];

void initModel3D();

int main(int argc, char* argv[]) {
    double start_time, end_time;
    initModel3D();
    for (int i = 0; i < 1; i++) {
        //start_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        Mat image = imread(".\\Data\\1.jpg");
        Mat image_gray;
        cvtColor(image, image_gray, COLOR_BGR2GRAY);
        image_gray.convertTo(image_gray, CV_32FC1); image_gray *= 1./255;
                
        imageParams ImgParams;
        ImgParams.height = image.rows;
        ImgParams.width = image.cols;

        PreFilter pF;
        candidate_corners = pF.preFilter(image_gray, 2 * number_of_corner_x * number_of_corner_y);

        FinalElection fE;
        cornerPoints = fE.finalElection(image_gray, candidate_corners);

        ArrayOrganization arrayOrg;
        int *matrix_p = arrayOrg.delaunayTriangulation(image_gray, cornerPoints);

        IdentifyMarker identify;
        identify.identifyMarker(image_gray, matrix_p, model_3D);

        //end_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        //cout << "FInal-election:" << (end_time - start_time) << endl;
        //waitKey(0);
        destroyAllWindows();
    }

    return 0;
}

void initModel3D() {

}