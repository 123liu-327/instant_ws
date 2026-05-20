#include <flow_end/generateLookupTable.h>
#include<flow_end/follow.h>

void generateLookupTable(float mapx[RESULT_ROW][RESULT_COL], float mapy[RESULT_ROW][RESULT_COL])
{
    ROS_INFO("      invMat:\n");
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            ROS_INFO("%f ", invMat[i][j]);
        }
        ROS_INFO("\n");
    }
    float mat[6];
    float a1, b1, c1, a2, b2, c2;
    for (int y = 0; y < RESULT_ROW; ++y)
    {
        for (int x = 0; x < RESULT_COL; ++x)
        {
            float loc_x = (float)((invMat[0][0] * x + invMat[0][1] * y + invMat[0][2]) / (invMat[2][0] * x + invMat[2][1] * y + invMat[2][2]));
            float loc_y = (float)((invMat[1][0] * x + invMat[1][1] * y + invMat[1][2]) / (invMat[2][0] * x + invMat[2][1] * y + invMat[2][2]));
            if (loc_y < USED_ROW)
            {
                mapx[y][x] = loc_x;
                mapy[y][x] = loc_y;
            }
        }
    }
    ROS_INFO("generateLookupTable end\n");
}

