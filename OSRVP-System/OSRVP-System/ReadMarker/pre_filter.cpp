#include "..\include\pre-filter.h"

using namespace std;

vector<Point> PreFilter::preFilter(Mat& img, int number) {
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
    float G_filtermin = score_sequence.ptr<float>(number)[1];
    for (int i = maskR; i < img_blur.rows - maskR; i++)
        for (int j = maskR; j < img_blur.cols - maskR; j++) {
    		if (G_score_after_NMS.ptr<float>(i)[j] < G_filtermin)
                G_score_after_NMS.ptr<float>(i)[j] = 0;
            else {
                candidate_corners.push_back(Point(j, i));
                //cout << j << ' ' << i << endl;
            }
        }	
    //imshow("G_score_after_NMS", G_score_after_NMS);
	return candidate_corners;
}
