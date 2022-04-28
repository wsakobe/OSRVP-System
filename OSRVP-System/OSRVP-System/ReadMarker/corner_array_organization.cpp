#include "../include/corner_array_organization.h"

using namespace std;
using namespace cv;

inline bool cmp(const Point& a, const Point& b)
{
    return a.x < b.x;
}

int *ArrayOrganization::delaunayTriangulation(Mat& img, vector<cornerInformation> cornerPoints){
   	Rect rect(0, 0, img.cols, img.rows);
    Subdiv2D subdiv(rect);

    for (int i = 0; i < cornerPoints.size(); i++)
    {
        e0 = 0, vertex = 0;
        subdiv.locate(cornerPoints[i].point_in_subpixel, e0, vertex);
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
                edge_list_ID.push_back(Point(org_ID, dst_ID));
                edge_list_ID.push_back(Point(dst_ID, org_ID));
            }
        }              
    }
    removeWrongEdges(img, cornerPoints);
    organizeCornersIntoArrays(img, cornerPoints);

    return &(matrix_with_ID[0][0][0]);
}

void ArrayOrganization::removeWrongEdges(Mat& img, vector<cornerInformation> cornerPoints){
    //Mat imgMark(img.rows, img.cols, CV_32FC3);
    //cvtColor(img, imgMark, COLOR_GRAY2RGB);

    memset(selection, true, sizeof(selection));

    sort(edge_list_ID.begin(), edge_list_ID.end(), cmp);
    
    last_ID = edge_list_ID[0].x;
    last_ID_pos = 0;
    dist_min = edgeDistance2(cornerPoints[edge_list_ID[0].x].point_in_subpixel, cornerPoints[edge_list_ID[0].y].point_in_subpixel);

    for (int i = 0; i < edge_list_ID.size() - 1; i++)
    {
        if (edge_list_ID[i + 1].x == last_ID) {
            dist_now = edgeDistance2(cornerPoints[edge_list_ID[i].x].point_in_subpixel, cornerPoints[edge_list_ID[i].y].point_in_subpixel);
            if (dist_now < dist_min)
                dist_min = dist_now;
        }
        else {
            for (int j = last_ID_pos; j <= i; j++)
                if (edgeDistance2(cornerPoints[edge_list_ID[j].x].point_in_subpixel, cornerPoints[edge_list_ID[j].y].point_in_subpixel) > 2 * dist_min) {
                    selection[j] = false;
                    for (int k = 0; k < edge_list_ID.size(); k++)
                        if ((edge_list_ID[k].y == edge_list_ID[j].x) && (edge_list_ID[k].x == edge_list_ID[j].y))
                            selection[k] = false;
                }
                    
            if (i != edge_list_ID.size() - 1) {
                last_ID = edge_list_ID[i + 1].x;
                last_ID_pos = i + 1;
                dist_min = MAX_DISTANCE;
            }            
        }
    }

    int cnt = 0;
    for (vector<Point>::iterator it = edge_list_ID.begin(); it != edge_list_ID.end();)
    {
        if (!selection[cnt++])
            it = edge_list_ID.erase(it);
        else
            it++;
    }
    /*
    for (int i = 0; i < edge_list_ID.size(); i++)
        line(imgMark, cornerPoints[edge_list_ID[i].x].point_in_subpixel, cornerPoints[edge_list_ID[i].y].point_in_subpixel, Scalar(100, 0, 100), 2, LINE_AA, 0);

    imshow("Delaunay", imgMark);
    waitKey(0);*/
}

void ArrayOrganization::organizeCornersIntoArrays(Mat& img, vector<cornerInformation> cornerPoints){
    //Mat imgMark(img.rows, img.cols, CV_32FC3);
    //cvtColor(img, imgMark, COLOR_GRAY2RGB);

    int matrix_number = 0;
    vector<matrixInform> corner_IDs(cornerPoints.size());
    vector<array<Point, 4>> direction(cornerPoints.size());
    memset(corner_visited, 0, sizeof(corner_visited));
    memset(matrix_with_ID, -1, sizeof(matrix_with_ID));
    
    for (int i = 0; i < edge_list_ID.size(); i++) {
        start_corner = 0;
        end_corner = 1;
        if ((!corner_visited[edge_list_ID[i].x]) && (cornerPoints[edge_list_ID[i].x].angle_black_edge > cornerPoints[edge_list_ID[i].x].angle_white_edge)) {
            q.push(edge_list_ID[i].x);
            corner_visited[edge_list_ID[i].x] = true;
            corner_IDs[edge_list_ID[i].x].mLabel = matrix_number;
            corner_IDs[edge_list_ID[i].x].mPos = Point(number_of_corner_x, number_of_corner_y);
            direction[edge_list_ID[i].x] = { Point(0, 1), Point(1, 0), Point(0, -1), Point(-1, 0) };
            matrix_with_ID[corner_IDs[edge_list_ID[i].x].mLabel][corner_IDs[edge_list_ID[i].x].mPos.x][corner_IDs[edge_list_ID[i].x].mPos.y] = edge_list_ID[i].x;

            while (start_corner != end_corner) {
                int corner_now = q.front();
                corner_visited[corner_now] = true;
                for (int j = 0; j < edge_list_ID.size(); j++) {
                    if (edge_list_ID[j].x == corner_now) {
                        if (!corner_visited[edge_list_ID[j].y]) {
                            Point p = directionJudge(fastAtan2(cornerPoints[edge_list_ID[j].y].point_in_subpixel.y - cornerPoints[corner_now].point_in_subpixel.y, cornerPoints[edge_list_ID[j].y].point_in_subpixel.x - cornerPoints[corner_now].point_in_subpixel.x), cornerPoints[corner_now], cornerPoints[edge_list_ID[j].y]);
                            
                            if ((p.x == -1) || (p.y == -1)) {
                                continue;
                            }

                            q.push(edge_list_ID[j].y);
                            corner_IDs[edge_list_ID[j].y].mLabel = corner_IDs[edge_list_ID[j].x].mLabel;
                            corner_IDs[edge_list_ID[j].y].mPos = direction[corner_now][p.x] + corner_IDs[corner_now].mPos;
                            matrix_with_ID[corner_IDs[edge_list_ID[j].y].mLabel][corner_IDs[edge_list_ID[j].y].mPos.x][corner_IDs[edge_list_ID[j].y].mPos.y] = edge_list_ID[j].y;

                            direction[edge_list_ID[j].y][p.y] = -direction[corner_now][p.x];
                            direction[edge_list_ID[j].y][(p.y + 2) % 4] = -direction[edge_list_ID[j].y][p.y];
                            if (cornerPoints[edge_list_ID[j].y].angle_black_edge > cornerPoints[edge_list_ID[j].y].angle_white_edge) {
                                direction[edge_list_ID[j].y][(p.y + 1) % 4] = Point(direction[edge_list_ID[j].y][p.y].y, -direction[edge_list_ID[j].y][p.y].x);
                            }
                            else {
                                direction[edge_list_ID[j].y][(p.y + 1) % 4] = Point(-direction[edge_list_ID[j].y][p.y].y, direction[edge_list_ID[j].y][p.y].x);
                            }
                            direction[edge_list_ID[j].y][(p.y + 3) % 4] = -direction[edge_list_ID[j].y][(p.y + 1) % 4];

                            corner_visited[edge_list_ID[j].y] = true;
                            end_corner++;
                        }
                    }
                    if (edge_list_ID[j].x > corner_now)
                        break;
                }
                start_corner++;
                q.pop();
            }
            matrix_number++;
        }
        if (matrix_number > 5) break;
    }
    
    /*
    for (int i = 0; i < cornerPoints.size(); i++) {
        std::stringstream ss;
        ss << '(' << corner_IDs[i].mPos.x << ", " << corner_IDs[i].mPos.y << ')';
        string s = ss.str();
        putText(imgMark, s, cornerPoints[i].point_in_subpixel + Point2f(2, 2), FONT_HERSHEY_SIMPLEX, 0.3, Scalar(0, 255, 0), 1.8);
    }

    imshow("Organization", imgMark);
    waitKey(0);*/
}

inline float ArrayOrganization::edgeDistance2(Point2f a, Point2f b) {
    return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

inline float ArrayOrganization::edgeAngle2(Point2f a, Point2f b) {
    float angle = fastAtan2(b.y - a.y, b.x - a.x);
    if ((angle < 270) && (angle > 90))
        angle -= 180;
    if (angle >= 270)
        angle -= 360;
    return angle;        
}

inline Point ArrayOrganization::directionJudge(float angle, cornerInformation corner_1, cornerInformation corner_2) {
    Point res = Point(-1, -1);
    if (angle >= 270)
        angle -= 360;
    if (angleJudge(angle, corner_1.angle_black_edge))         res.x = 0;
    if (angleJudge(angle, corner_1.angle_white_edge))         res.x = 1;
    if (angleJudge(angle, corner_1.angle_black_edge + 180))   res.x = 2;
    if (angleJudge(angle, corner_1.angle_white_edge + 180))   res.x = 3;

    if (angleJudge(angle, corner_2.angle_black_edge))         res.y = 2;
    if (angleJudge(angle, corner_2.angle_white_edge))         res.y = 3;
    if (angleJudge(angle, corner_2.angle_black_edge + 180))   res.y = 0;
    if (angleJudge(angle, corner_2.angle_white_edge + 180))   res.y = 1;

    return res;
}

inline bool ArrayOrganization::angleJudge(float angle1, float angle2) {
    if (abs(angle1 - angle2) < maxCornerAngleInOrg)
        return true;
    else
        return false;
}