#include "..\include\final_election.h"

using namespace std;

FinalElection::FinalElection() {
    int maskL = 11;
    //Search for the template in the dic. If not exist, generate once.
    String tmpFile = "template";
    tmpFile.append(std::to_string(maskL));
    tmpFile.append(".bmp");

    struct stat buffer;
    if (stat(tmpFile.c_str(), &buffer) != 0)
    {
        Mat tmpMSAA, tmpCrop;
        tmpMSAA = Mat::zeros(10 * maskL, 10 * maskL, CV_32FC1);
        tmp = Mat::zeros(36 * maskL, 36 * maskL, CV_32FC1);
        for (float B = 0, angleB = 0; B < 36; angleB = angleB + 5, ++B) {
            for (float W = 0, angleW = 0; W < 36; angleW = angleW + 5, ++W) {
                float ix = 0.5 - float(tmpMSAA.cols) / 2;
                float iy;
                for (int x = 0; x < tmpMSAA.cols; ++x, ++ix) {
                    iy = float(tmpMSAA.rows) / 2 - 0.5;
                    for (int y = 0; y <= x; ++y, --iy) {
                        float temp = (atan2(ix, iy)) / CV_PI * 180 + 45;

                        if (angleB == angleW) continue;
                        if (temp > angleW && temp < angleB) {
                            tmpMSAA.at<float>(y, x) = 1;
                            tmpMSAA.at<float>(tmpMSAA.rows - y - 1, tmpMSAA.cols - x - 1) = 1;
                        }
                        else if (angleB < angleW && (temp<angleB || temp>angleW)) {
                            tmpMSAA.at<float>(y, x) = 1;
                            tmpMSAA.at<float>(tmpMSAA.rows - y - 1, tmpMSAA.cols - x - 1) = 1;
                        }
                        else {
                            tmpMSAA.at<float>(y, x) = 0;
                            tmpMSAA.at<float>(tmpMSAA.rows - y - 1, tmpMSAA.cols - x - 1) = 0;
                        }
                    }
                }
                tmpCrop = tmp(Rect(B * maskL, W * maskL, maskL, maskL));
                resize(tmpMSAA, tmpCrop, Size(maskL, maskL), 0, 0, INTER_AREA);
            }
        }
        imwrite(tmpFile, 255 * tmp);
        tmpMSAA.release();
        tmpCrop.release();
    }
    else
    {
        tmp = imread(tmpFile);
        cvtColor(tmp, tmp, COLOR_BGR2GRAY);
        tmp.convertTo(tmp, CV_32FC1);
        tmp = tmp / 255;
    }

    //Generate the order of the neighboring surface
    surface_temp_x = Mat::zeros(maskSurfaceL, maskSurfaceL, CV_32FC1);
    surface_temp_y = Mat::zeros(maskSurfaceL, maskSurfaceL, CV_32FC1);
    surface_temp   = Mat::zeros(maskSurfaceL, maskSurfaceL, CV_32FC1);
    B = Mat::zeros(maskSurfaceL * maskSurfaceL, 1, CV_32FC1);

    for (int i = 0; i < maskSurfaceL; i++)
        for (int j = 0; j < maskSurfaceL; j++) {
            surface_temp_x.at<float>(i, j) = i;
        }
    transpose(surface_temp_x, surface_temp_y);
    surface_temp_x = surface_temp_x.reshape(1, maskSurfaceL * maskSurfaceL);
    surface_temp_y = surface_temp_y.reshape(1, maskSurfaceL * maskSurfaceL);
    hconcat(surface_temp_x, surface_temp_y, surface_temp);
}

FinalElection::~FinalElection() {
    tmp.release();
}

vector<cornerInformation> FinalElection::finalElection(Mat& img, vector<Point> corners) {
    cornerPoints = subpixelRefinement(img, corners);
    cornerPoints = fitQuadraticSurface(img, cornerPoints);
    cornerPoints = templateMatching(img, cornerPoints);
    
    return cornerPoints;
}

vector<cornerInformation> FinalElection::subpixelRefinement(Mat& img, vector<Point> corners) {
    cornerInformation cur;
    for (int i = 0; i < corners.size(); i++) {
        cur.point_in_pixel.x = corners[i].x;
        cur.point_in_pixel.y = corners[i].y;

        Rect rect_neighbor(corners[i].x - maskSurface, corners[i].y - maskSurface, maskSurfaceL, maskSurfaceL);
        img_neighbor = img(rect_neighbor);

        Scharr(img_neighbor, grad_neighbor_x, CV_32FC1, 1, 0);
        Scharr(img_neighbor, grad_neighbor_y, CV_32FC1, 0, 1);
        grad_neighbor_x = grad_neighbor_x.reshape(1, maskSurfaceL * maskSurfaceL);
        grad_neighbor_y = grad_neighbor_y.reshape(1, maskSurfaceL * maskSurfaceL);
        hconcat(grad_neighbor_x, grad_neighbor_x, grad_neighbor);

        for (int j = 0; j < grad_neighbor.rows; j++) {
            grad_row = grad_neighbor.rowRange(j, j + 1);
            surface_row = surface_temp.rowRange(j, j + 1);
            result = surface_row.dot(grad_row);
            B.at<float>(j, 0) = (float)result;
        }

        solve(grad_neighbor, B, subpixel, DECOMP_SVD);
        cur.point_in_subpixel.x = subpixel.at<float>(0, 0) + corners[i].x;
        cur.point_in_subpixel.y = subpixel.at<float>(1, 0) + corners[i].y;
        cout << cur.point_in_subpixel.x << ' ' << cur.point_in_subpixel.y << endl;
        cornerPoints.push_back(cur);
    }
    
    return cornerPoints;
}

vector<cornerInformation> FinalElection::fitQuadraticSurface(Mat& img, vector<cornerInformation> cornerPoints) {
    for (int i = 0; i < cornerPoints.size(); i++) {
        Rect rect(cornerPoints[i].point_in_pixel.x - maskSurface, cornerPoints[i].point_in_pixel.y - maskSurface, maskSurface * 2 + 1, maskSurface * 2 + 1);
        Mat image_roi = img(rect);
    }
    
    return cornerPoints;
}

vector<cornerInformation> FinalElection::templateMatching(Mat& img, vector<cornerInformation> cornerPoints) {
    
    
    return cornerPoints;
}