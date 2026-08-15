#ifndef FLOW_END_PROCESS_IMAGE_H
#define FLOW_END_PROCESS_IMAGE_H

#include<flow_end/follow.h>
#include<flow_end/Findline_Adaptive.h>
#include<flow_end/Point_Process.h>
#include<flow_end/track_line.h>

struct ForwardCrossbarResult {
    bool found;
    int center_x;
    int center_y;
    int width_px;
    float map_x;
    float map_y;
    float long_m;
    float lat_m;
};

extern ForwardCrossbarResult forward_crossbar_result;
//综合处理图像
void process_image();

// Scan the lane crossbar directly ahead and update forward_crossbar_result.
bool detect_forward_crossbar();

#endif
