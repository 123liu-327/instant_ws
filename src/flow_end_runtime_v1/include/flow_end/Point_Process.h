#ifndef FLOW_END_POINT_PROCESS_H
#define FLOW_END_POINT_PROCESS_H
#include <flow_end/follow.h>

//平滑处理，消除部分噪声
void blur_points(float pts_in[][2], int num, float pts_out[][2], int kernel);
//等距采样
void resample_points(float pts_in[][2], int num1, float pts_out[][2], int *num2, float dist);
//局部角度估计
void local_angle_points(float pts_in[][2], int num, float angle_out[], int dist);
// 角度变化非极大值抑制
void nms_angle(float angle_in[], int num, float angle_out[], int kernel);

#endif 