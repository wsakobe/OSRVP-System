#include "..\include\pre-filter.h"

void PreFilter::preFilter(Mat& img, int number) {
    std::vector<Point> curPos;
    GaussianBlur(img, img, Size(kernal_size, kernal_size), sigma);
    Scharr(img, Gx, CV_32FC1, 1, 0);
    Scharr(img, Gy, CV_32FC1, 0, 1);
    Gx *= 1. / 255;
    Gy *= 1. / 255;
    Scharr(Gx, Gxx, CV_32FC1, 1, 0);
    Scharr(Gy, Gyy, CV_32FC1, 0, 1);
    Scharr(Gx, Gxy, CV_32FC1, 0, 1);

    G_score = Gxy.mul(Gxy) - Gxx.mul(Gyy);
    dilate(G_score, G_score_after_NMS, Mat());
    std::cout << Gxx.ptr<float>(513)[547] << std::endl;
    
    for (int i = maskR; i < img.cols - maskR; i++)
    	for (int j = maskR; j < img.rows - maskR; j++)
    		if (G_score.ptr<float>(i)[j] != G_score_after_NMS.ptr<float>(i)[j])
                G_score_after_NMS.ptr<float>(i)[j] = 0;

    G_score_after_NMS.copyTo(score_sequence);
    score_sequence = score_sequence.reshape(1, img.cols * img.rows);
    //sort(score_sequence, score_sequence, SORT_EVERY_COLUMN + SORT_DESCENDING);
    float G_filtermin = score_sequence.ptr<float>(number)[1];
    for (int i = maskR; i < img.cols - maskR; i++)
        for (int j = maskR; j < img.rows - maskR; j++) {
    		if (G_score_after_NMS.ptr<float>(i)[j] < G_filtermin)
                G_score_after_NMS.ptr<float>(i)[j] = 0;
    		else 
                curPos.push_back(Point(j, i));
    	}	
    imshow("G_score_after_NMS", G_score * 255);
	return;
}
