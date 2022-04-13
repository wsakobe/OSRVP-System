#ifndef INCLUDE_CORNER_ARRAY_ORGANIZATION_H_
#define INCLUDE_CORNER_ARRAY_ORGANIZATION_H_

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "final_election.h"

#define MAX_DISTANCE 10000.0;

struct matrixInform
{
	int     mLabel = -1;      // ËùÊô¾ØÕó±àºÅ
	Point   mPos;           // ¾ØÕó×ø±ê
};

class ArrayOrganization {
public:
	vector<matrixInform> matrix_array_pos;
	int *delaunayTriangulation(Mat& img, vector<cornerInformation> cornerPoints);
	void removeWrongEdges(Mat& img, vector<cornerInformation> cornerPoints);
	void organizeCornersIntoArrays(Mat& img, vector<cornerInformation> cornerPoints);

	vector<Point> edge_list_ID;
	
	int matrix_with_ID[5][2 * 10][2 * 10]; //matrix_with_ID[labelNum][i][j] labelNum is the number of Markers, this matrix records the IDs in each coordinate.

private:
	Subdiv2D subdiv_delaunay;
	int e0 = 0, vertex = 0;
	Point2f org, dst;
	vector<Vec4f> edge_list;

	float edgeDistance2(Point2f a, Point2f b);
	float edgeAngle2(Point2f a, Point2f b);
	Point directionJudge(float angle, cornerInformation corner_1, cornerInformation corner_2);
	bool  angleJudge(float angle1, float angle2);

	const float maxCornerDistance = 50.0, maxCornerAngle = 15.0, maxCornerAngleInOrg = 25.0;
	float edge_angle, angle_oppo;
	float dist_now, dist_min = 1000.0;

	bool selection[8 * 100], corner_visited[100];

	int org_ID, dst_ID, last_ID, last_ID_pos;
	int start_corner, end_corner;

	queue <int> q;
};

#endif