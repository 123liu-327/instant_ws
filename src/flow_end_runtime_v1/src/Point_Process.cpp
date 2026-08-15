#include <flow_end/Point_Process.h>

// 平滑处理
void blur_points(float pts_in[][2], int num, float pts_out[][2], int kernel)
{
    assert(kernel % 2 == 1);
    int half = kernel / 2;

    for (int i = 0; i < num; i++)
    {
        pts_out[i][0] = pts_out[i][1] = 0;
        for (int j = -half; j <= half; j++)
        {
            int idx = clip(i + j, 0, num - 1);
            pts_out[i][0] += pts_in[idx][0] * (half + 1 - std::abs(j));
            pts_out[i][1] += pts_in[idx][1] * (half + 1 - std::abs(j));
        }
        pts_out[i][0] /= (2 * half + 2) * (half + 1) / 2.0f;
        pts_out[i][1] /= (2 * half + 2) * (half + 1) / 2.0f;
    }
}

// 等距采样
void resample_points(float pts_in[][2], int num1, float pts_out[][2], int *num2, float dist)
{
    int len = 0;
    float remain = 0;
    for (int i = 0; i < num1 - 1 && len < *num2; i++)
    {
        float x0 = pts_in[i][0];
        float y0 = pts_in[i][1];
        float dx = pts_in[i + 1][0] - x0;
        float dy = pts_in[i + 1][1] - y0;
        float dn = std::sqrt(dx * dx + dy * dy);
        dx /= dn;
        dy /= dn;

        while (remain < dn && len < *num2)
        {
            x0 += dx * remain;
            pts_out[len][0] = x0;
            y0 += dy * remain;
            pts_out[len][1] = y0;

            len++;
            dn -= remain;
            remain = dist;
        }
        remain -= dn;
    }
    *num2 = len;
}
//局部角度估计
void local_angle_points(float pts_in[][2], int num, float angle_out[], int dist)
{
    for (int i = 0; i < num; i++)
    {
        if (i <= 0 || i >= num - 1)
        {
            angle_out[i] = 0;
            continue;
        }
        float dx1 = pts_in[i][0] - pts_in[clip(i - dist, 0, num - 1)][0];
        float dy1 = pts_in[i][1] - pts_in[clip(i - dist, 0, num - 1)][1];
        float dn1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
        float dx2 = pts_in[clip(i + dist, 0, num - 1)][0] - pts_in[i][0];
        float dy2 = pts_in[clip(i + dist, 0, num - 1)][1] - pts_in[i][1];
        float dn2 = std::sqrt(dx2 * dx2 + dy2 * dy2);
        float c1 = dx1 / dn1;
        float s1 = dy1 / dn1;
        float c2 = dx2 / dn2;
        float s2 = dy2 / dn2;
        angle_out[i] = std::atan2(c1 * s2 - c2 * s1, c2 * c1 + s2 * s1);
    }
}
// 角度变化非极大值抑制
void nms_angle(float angle_in[], int num, float angle_out[], int kernel)
{
    assert(kernel % 2 == 1);
    int half = kernel / 2;

    for (int i = 0; i < num; i++)
    {
        angle_out[i] = angle_in[i];
        for (int j = -half; j <= half; j++)
        {
            if (std::abs(angle_in[clip(i + j, 0, num - 1)]) > std::abs(angle_out[i]))
            {
                angle_out[i] = 0;
                break;
            }
        }
    }
}