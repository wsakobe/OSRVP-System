#include "..\include\pre-filter.h"

using namespace std;

vector<cornerPreInfo> PreFilter::preFilter(Mat& img, int number) {
    //Mat imgMark(img.rows, img.cols, CV_32FC3);
    //cvtColor(img, imgMark, COLOR_GRAY2RGB);

    GaussianBlur(img, img_blur, Size(kernal_size, kernal_size), sigma);
    Scharr(img_blur, Gx, CV_32FC1, 1, 0);
    Scharr(img_blur, Gy, CV_32FC1, 0, 1);

    Scharr(Gx, Gxx, CV_32FC1, 1, 0);
    Scharr(Gy, Gyy, CV_32FC1, 0, 1);
    Scharr(Gx, Gxy, CV_32FC1, 0, 1);

    G_score = Gxy.mul(Gxy) - Gxx.mul(Gyy);
    dilate(G_score, G_score_after_NMS, Mat());
    
    for (int i = maskR; i < img_blur.rows - maskR; i++)
    	for (int j = maskR; j < img_blur.cols - maskR; j++)
    		if (G_score.ptr<float>(i)[j] != G_score_after_NMS.ptr<float>(i)[j])
                G_score_after_NMS.ptr<float>(i)[j] = 0;

    G_score_after_NMS.copyTo(score_sequence);
    score_sequence = score_sequence.reshape(1, img_blur.cols * img_blur.rows);
    cv::sort(score_sequence, score_sequence, SORT_EVERY_COLUMN + SORT_DESCENDING);

    G_filtermax = score_sequence.ptr<float>(0)[1];
    G_filtermin = score_sequence.ptr<float>(number)[1];

    for (int i = maskR; i < img_blur.rows - maskR; i++)
        for (int j = maskR; j < img_blur.cols - maskR; j++) {
    		if (G_score_after_NMS.ptr<float>(i)[j] < G_filtermin)
                G_score_after_NMS.ptr<float>(i)[j] = 0;
            else {
                temporal_corner.corner_position = Point(j ,i);
                temporal_corner.response_score = G_score_after_NMS.ptr<float>(i)[j] / G_filtermax;
                corners.push_back(temporal_corner);

                //circle(imgMark, Point(j, i), 3, Scalar(255, 0, 0), -1);
                //std::stringstream ss;
                //ss << std::setprecision(4) << G_score_after_NMS.ptr<float>(i)[j];
                //string s = ss.str();
                //putText(imgMark, s, Point(j, i) + Point(2, 2), FONT_ITALIC, 0.3, Scalar(0, 255, 0));
            }
        }	
    //imshow("imgMark", imgMark);
    //waitKey(0);
	return corners;
}
