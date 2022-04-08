#ifndef INCLUDE_CORNER_ARRAY_ORGANIZATION_H_
#define INCLUDE_CORNER_ARRAY_ORGANIZATION_H_

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "final_election.h"

struct matrixInform
{
	int     mLabel = -1;      // ����������
	Point   mPos;           // ��������
};

class ArrayOrganization {
public:
	vector<matrixInform> matrix_array_pos;
	void delaunayTriangulation(Mat& img, vector<cornerInformation> cornerPoints);
	void removeWrongEdges(Subdiv2D subdiv_delaunay, vector<cornerInformation> cornerPoints);
	void organizeCornersIntoArrays(Subdiv2D subdiv_delaunay, vector<cornerInformation> cornerPoints);

	vector<Point> edge_list_ID;

private:
	Subdiv2D subdiv_delaunay;
	int e0 = 0, vertex = 0;
	Point2f org, dst;
	vector<Vec4f> edge_list;

	float edgeDistance2(Point2f a, Point2f b);
	float edgeAngle2(Point2f a, Point2f b);
	const float maxCornerDistance = 50.0, maxCornerAngle = 18.0;
	float edge_angle;

	int org_ID, dst_ID;
};

#endif