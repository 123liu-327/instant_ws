#ifndef FLOW_END_PROCESS_IMAGE_H
#define FLOW_END_PROCESS_IMAGE_H

#include<flow_end/follow.h>
#include<flow_end/Findline_Adaptive.h>
#include<flow_end/Point_Process.h>
#include<flow_end/track_line.h>
//综合处理图像
void process_image();

// 正前方横线检测结果。center_* 是图像坐标；map_* 是 point_map 映射后的平面坐标；
// long_m/lat_m 是沿用当前工程 pixel_per_meter 换算出的近似车体坐标。
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

// 独立横线检测函数：只在调用时更新 forward_crossbar_result，
// 不会被 process_image() 自动执行，也不改变旧扫线结果。
bool detect_forward_crossbar();

#endif
