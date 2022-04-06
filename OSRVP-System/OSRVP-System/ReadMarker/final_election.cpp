#include "..\include\final_election.h"

using namespace std;

FinalElection::FinalElection() {
    int maskL = 13;
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

    //Generate the order of the hypersurface neighborhood
    hypersurface_temp_x2 = Mat::zeros(maskSurfaceL * maskSurfaceL, 1, CV_32FC1);
    hypersurface_temp_y2 = Mat::zeros(maskSurfaceL * maskSurfaceL, 1, CV_32FC1);
    hypersurface_temp_xy = Mat::zeros(maskSurfaceL * maskSurfaceL, 1, CV_32FC1);
    hypersurface_temp_x  = Mat::zeros(maskSurfaceL * maskSurfaceL, 1, CV_32FC1);
    hypersurface_temp_y  = Mat::zeros(maskSurfaceL * maskSurfaceL, 1, CV_32FC1);
    hypersurface_temp    = Mat::zeros(maskSurfaceL * maskSurfaceL, 1, CV_32FC1);
    for (int i = 0; i < maskSurfaceL; i++)
        for (int j = 0; j < maskSurfaceL; j++) {
            hypersurface_temp_x2.at<float>(i * maskSurfaceL + j, 0) = i * i;
            hypersurface_temp_y2.at<float>(i * maskSurfaceL + j, 0) = j * j;
            hypersurface_temp_xy.at<float>(i * maskSurfaceL + j, 0) = i * j;
            hypersurface_temp_x.at<float>(i * maskSurfaceL + j, 0) = i;
            hypersurface_temp_y.at<float>(i * maskSurfaceL + j, 0) = j;
        }
    hconcat(hypersurface_temp_x2, hypersurface_temp_xy, hypersurface_temp);
    hconcat(hypersurface_temp, hypersurface_temp_y2, hypersurface_temp);
    hconcat(hypersurface_temp, hypersurface_temp_x, hypersurface_temp);
    hconcat(hypersurface_temp, hypersurface_temp_y, hypersurface_temp);
    hconcat(hypersurface_temp, Mat::ones(maskSurfaceL * maskSurfaceL, 1, CV_32FC1), hypersurface_temp);
}

FinalElection::~FinalElection() {
    tmp.release();
}

vector<cornerInformation> FinalElection::finalElection(Mat& img, vector<Point> corners) {
    subpixelRefinement(img, corners);
    fitQuadraticSurface(img);
    templateMatching(img);
    
    return cornerPoints;
}

void FinalElection::subpixelRefinement(Mat& img, vector<Point> corners) {
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
        cur.point_in_subpixel.x = subpixel.at<float>(0, 0) + corners[i].x - maskSurface;
        cur.point_in_subpixel.y = subpixel.at<float>(1, 0) + corners[i].y - maskSurface;
        cornerPoints.push_back(cur);
    }
    
    return;
}

void FinalElection::fitQuadraticSurface(Mat& img) {
    for (int i = 0; i < cornerPoints.size(); i++) {
        Rect rect(cornerPoints[i].point_in_pixel.x - maskSurface, cornerPoints[i].point_in_pixel.y - maskSurface, maskSurfaceL, maskSurfaceL);
        img_hypersurface = img(rect).clone();
        img_hypersurface = img_hypersurface.reshape(1, maskSurfaceL * maskSurfaceL);
        solve(hypersurface_temp, img_hypersurface, hypersurface_coeffs, DECOMP_SVD);
        
        coeffs = (Mat_<float>(3, 1) << hypersurface_coeffs.at<float>(0, 0), -1 * hypersurface_coeffs.at<float>(1, 0), hypersurface_coeffs.at<float>(2, 0));
        solvePoly(coeffs, roots);
        
        angle1 = atan(roots.at<float>(0, 0)) * 180.0 / PI;
        angle2 = atan(roots.at<float>(1, 0)) * 180.0 / PI;
        
        if ((angle1 * angle2 * hypersurface_coeffs.at<float>(0, 0)) < 0) {
            cornerPoints[i].angle_white_edge = max(angle1, angle2);
            cornerPoints[i].angle_black_edge = min(angle1, angle2);
        }
        else {
            cornerPoints[i].angle_white_edge = min(angle1, angle2);
            cornerPoints[i].angle_black_edge = max(angle1, angle2);
        }
        //cout << i << ' ' << coeffs.at<float>(0, 0) << ' ' << coeffs.at<float>(1, 0) << ' ' << coeffs.at<float>(2, 0) << ' ' << roots.at<float>(0, 0) << ' ' << roots.at<float>(1, 0) << ' ' << cornerPoints[i].angle_white_edge << ' ' << cornerPoints[i].angle_black_edge << endl;
    }
    
    return;
}

void FinalElection::templateMatching(Mat& img) {
    Mat imgMark(img.rows, img.cols, CV_32FC3);
    cvtColor(img, imgMark, COLOR_GRAY2RGB);

    for (int i = 0; i < cornerPoints.size(); i++) {
        //edge_angle = cornerPoints[i].angle_black_edge - cornerPoints[i].angle_white_edge;
        //direction_angle = (cornerPoints[i].angle_black_edge + cornerPoints[i].angle_white_edge) / 2;
        
        edgeIdx   = round((cornerPoints[i].angle_black_edge + 135) / 5);
        directIdx = round((cornerPoints[i].angle_white_edge + 135) / 5);
        
        if (edgeIdx < 0 || edgeIdx > 35) edgeIdx = 0;
        if (directIdx < 0 || directIdx > 35) directIdx = 0;
        
        Mat tmpCrop(tmp, Rect(edgeIdx * maskTemp, directIdx * maskTemp, maskTemp, maskTemp));
        Mat crop(img, Rect(cornerPoints[i].point_in_pixel.x - maskTemR, cornerPoints[i].point_in_pixel.y - maskTemR, maskTemp, maskTemp));

        Scalar meanTmp, meanCrop, stdTmp, stdCrop;
        meanStdDev(tmpCrop, meanTmp, stdTmp);
        meanStdDev(crop, meanCrop, stdCrop);

        float covar = (tmpCrop - meanTmp).dot(crop - meanCrop) / (maskTemp * maskTemp);
        cornerPoints[i].response_score = covar / (stdTmp[0] * stdCrop[0]);
        if (cornerPoints[i].response_score > response_score_max) {
            response_score_max = cornerPoints[i].response_score;
        }
    }
    /*
    vector<cornerInformation>::iterator iter;
    for (iter = cornerPoints.begin(); iter != cornerPoints.end(); iter++) {
        if ((*iter).response_score < response_score_max - T_response) {
            cornerPoints.erase(iter);
        }
    }*/
    for (vector<cornerInformation>::iterator it = cornerPoints.begin(); it != cornerPoints.end();)
    {
        if ((*it).response_score < response_score_max - T_response)
            it = cornerPoints.erase(it);
        else
            it++;
    }

    for (int i = 0; i < cornerPoints.size(); i++) {
        circle(imgMark, cornerPoints[i].point_in_subpixel, 3, Scalar(255, 0, 0), -1);
        std::stringstream ss;
        ss << std::setprecision(2) << cornerPoints[i].response_score;
        string s = ss.str();
        putText(imgMark, s, cornerPoints[i].point_in_subpixel + Point2f(2, 2), FONT_ITALIC, 0.3, Scalar(0, 255, 0));
    }
    imshow("imgMark", imgMark);
    waitKey(0);

    return;
}