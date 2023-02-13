#pragma once
#ifndef INCLUDE_INIT_H_
#define INCLUDE_INIT_H_

#include "config.h"

using namespace cv;
using namespace std;

void initData();

//ReadMarker Preparation
extern int camera_number, tracking_number;
extern vector<CameraParams> camera_parameters, endo_parameter;
extern CameraParams cam;
extern vector<ModelInfo> model_3D;
extern int model_num, point_num, value;
extern vector<vector<DynamicROIBox>> Box; // 0 - robotROI; 1 - openerROI; 2 - endoscopeROI
extern imageParams ImgParams, ImgParamsOri, ImgParamsEndo;
extern int number_of_corner_x_input, number_of_corner_y_input;

#endif