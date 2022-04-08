#include "../include/corner_array_organization.h"

using namespace std;
using namespace cv;

bool cmp(const Point& a, const Point& b)
{
    return a.x < b.x;
}

void ArrayOrganization::delaunayTriangulation(Mat& img, vector<cornerInformation> cornerPoints){
    Mat imgMark(img.rows, img.cols, CV_32FC3);
    cvtColor(img, imgMark, COLOR_GRAY2RGB);

	Rect rect(0, 0, img.cols, img.rows);
    Subdiv2D subdiv(rect);

    for (int i = 0; i < cornerPoints.size(); i++)
    {
        e0 = 0, vertex = 0;
        subdiv.locate(cornerPoints[i].point_in_subpixel, e0, vertex);
        circle(imgMark, cornerPoints[i].point_in_subpixel, 3, Scalar(0, 255, 0), -1);
        if (e0 > 0)
        {
            int e = e0;
            do
            {
                e = subdiv.getEdge(e, Subdiv2D::PREV_AROUND_ORG);
            } while (e != e0);
        }
        subdiv.insert(cornerPoints[i].point_in_subpixel);
    }
    subdiv.getEdgeList(edge_list);

    for (int i = 0; i < edge_list.size(); i++) {
        org = Point2f(edge_list[i][0], edge_list[i][1]);
        dst = Point2f(edge_list[i][2], edge_list[i][3]);
        if (edgeDistance2(org, dst) < maxCornerDistance) {
            for (int j = 0; j < cornerPoints.size(); j++) {
                if ((abs(cornerPoints[j].point_in_subpixel.x - org.x) < 1e-3) && (abs(cornerPoints[j].point_in_subpixel.y - org.y) < 1e-3)) {
                    org_ID = j;
                    break;
                }
            }
            for (int j = 0; j < cornerPoints.size(); j++) {
                if ((abs(cornerPoints[j].point_in_subpixel.x - dst.x) < 1e-3) && (abs(cornerPoints[j].point_in_subpixel.y - dst.y) < 1e-3)) {
                    dst_ID = j;
                    break;
                }
            }
            edge_angle = edgeAngle2(cornerPoints[org_ID].point_in_subpixel, cornerPoints[dst_ID].point_in_subpixel);
            if ((abs(edge_angle - cornerPoints[org_ID].angle_black_edge) < maxCornerAngle) || (abs(edge_angle - cornerPoints[org_ID].angle_white_edge) < maxCornerAngle)){
                line(imgMark, org, dst, Scalar(100, 0, 100), 2, LINE_AA, 0);
                edge_list_ID.push_back(Point(org_ID, dst_ID));
                edge_list_ID.push_back(Point(dst_ID, org_ID));
            }
        }              
    }
    
    sort(edge_list_ID.begin(), edge_list_ID.end(), cmp);


    imshow("Delaunay", imgMark);
    waitKey(0);
}

void ArrayOrganization::removeWrongEdges(Subdiv2D subdiv_delaunay, vector<cornerInformation> cornerPoints){

}

void ArrayOrganization::organizeCornersIntoArrays(Subdiv2D subdiv_delaunay, vector<cornerInformation> cornerPoints){

}

float ArrayOrganization::edgeDistance2(Point2f a, Point2f b) {
    return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

float ArrayOrganization::edgeAngle2(Point2f a, Point2f b) {
    float angle = fastAtan2(b.y - a.y, b.x - a.x);
    if ((angle < 270) && (angle > 90))
        angle -= 180;
    if (angle >= 270)
        angle -= 360;
    return angle;        
}