#include <flow_end/corner_move.h>
#include <flow_end/follow.h>
void corner_move(float pts_in[][2], float corner_dot[], int corner_idx, float dist)
{
    int approx_num = 3;
    float dx = pts_in[clip(corner_idx, 0, corner_idx)][0] - pts_in[clip(corner_idx - approx_num, 0, corner_idx)][0];
    float dy = pts_in[clip(corner_idx, 0, corner_idx)][1] - pts_in[clip(corner_idx - approx_num, 0, corner_idx)][1];
    float dn = sqrt(dx * dx + dy * dy);
    dx /= dn;
    dy /= dn;
    corner_dot[0] = pts_in[corner_idx][0] - dy * dist;
    corner_dot[1] = pts_in[corner_idx][1] + dx * dist;
}