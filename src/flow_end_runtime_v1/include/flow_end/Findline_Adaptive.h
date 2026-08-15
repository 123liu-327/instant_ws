#ifndef FLOW_END_FINDLINE_ADAPTIVE_H
#define FLOW_END_FINDLINE_ADAPTIVE_H
#include <flow_end/follow.h>
//左手迷宫
void findline_lefthand_adaptive(image_t *img, int block_size, float clip_value, int x, int y, int pts[][2], int *num);
//右手迷宫
void findline_righthand_adaptive(image_t *img, int block_size, float clip_value, int x, int y, int pts[][2], int *num);
#endif