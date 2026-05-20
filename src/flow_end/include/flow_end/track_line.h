#ifndef FLOW_END_TRACK_LINE_H
#define FLOW_END_TRACK_LINE_H
#include <flow_end/follow.h>
//左巡线
void track_leftline(float pts_in[][2], int num, float pts_out[][2], int approx_num, float dist);
//右巡线
void track_rightline(float pts_in[][2], int num, float pts_out[][2], int approx_num, float dist);

#endif