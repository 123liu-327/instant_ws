#include <flow_end/track_line.h>

//左巡线
void track_leftline(float pts_in[][2], int num, float pts_out[][2], int approx_num, float dist)
{
    pts_out[0][0] = 320;
    pts_out[0][1] = 480 + 50;
    for (int i = 1; i < num; i++)
    {
        float dx = pts_in[clip(i + approx_num, 0, num - 1)][0] - pts_in[clip(i - approx_num, 0, num - 1)][0];
        float dy = pts_in[clip(i + approx_num, 0, num - 1)][1] - pts_in[clip(i - approx_num, 0, num - 1)][1];
        float dn = std::sqrt(dx * dx + dy * dy);
        dx /= dn;
        dy /= dn;
        pts_out[i][0] = pts_in[i][0] - dy * dist;
        pts_out[i][1] = pts_in[i][1] + dx * dist;
    }
}
//右巡线
void track_rightline(float pts_in[][2], int num, float pts_out[][2], int approx_num, float dist)
{
    pts_out[0][0] = 320;
    pts_out[0][1] = 480 + 50;
    for (int i = 1; i < num; i++)
    {
        float dx = pts_in[clip(i + approx_num, 0, num - 1)][0] - pts_in[clip(i - approx_num, 0, num - 1)][0];
        float dy = pts_in[clip(i + approx_num, 0, num - 1)][1] - pts_in[clip(i - approx_num, 0, num - 1)][1];
        float dn = std::sqrt(dx * dx + dy * dy);
        dx /= dn;
        dy /= dn;
        pts_out[i][0] = pts_in[i][0] + dy * dist;
        pts_out[i][1] = pts_in[i][1] - dx * dist;
    }
}