#include "include\pre-filter.h"

using namespace cv;
using namespace std;

const Mat CamIntrinsicLeft = (Mat_<double>(3, 3) << 1.0, 0, 0,
                                                    0, 1.0, 0,
                                                    0, 0, 1);
const Mat DistCoeffLeft = (Mat_<double>(1, 5) << 0, 0, 0, 0, 0);
const int numberOfCorner = 10 * 10;
vector<Point> candidate_corners;

int main(int argc, char* argv[]) {
    Mat image = imread(".\\Data\\1.jpg");
    Mat image_gray;
    cvtColor(image, image_gray, COLOR_BGR2GRAY);
    image_gray.convertTo(image_gray, CV_32FC1); image_gray *= 1./255;
    
    PreFilter pF;
    candidate_corners = pF.preFilter(image_gray, 2 * numberOfCorner);

    FinalElection fE;
    fE.finalElection(image_gray, candidate_corners);

    waitKey(0);
    destroyAllWindows();
    return 0;
}