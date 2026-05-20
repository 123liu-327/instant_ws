#include <flow_end/follow.h>
#include <flow_end/ImagePerspectiveInit.h>



void invertMatrix(const double inputMat[3][3], double outputMat[3][3])
{
    //         ʽ
    double det = inputMat[0][0] * (inputMat[1][1] * inputMat[2][2] - inputMat[1][2] * inputMat[2][1]) -
                 inputMat[0][1] * (inputMat[1][0] * inputMat[2][2] - inputMat[1][2] * inputMat[2][0]) +
                 inputMat[0][2] * (inputMat[1][0] * inputMat[2][1] - inputMat[1][1] * inputMat[2][0]);
    //           ÿ  Ԫ
    outputMat[0][0] = (inputMat[1][1] * inputMat[2][2] - inputMat[1][2] * inputMat[2][1]) / det;
    outputMat[0][1] = (inputMat[0][2] * inputMat[2][1] - inputMat[0][1] * inputMat[2][2]) / det;
    outputMat[0][2] = (inputMat[0][1] * inputMat[1][2] - inputMat[0][2] * inputMat[1][1]) / det;

    outputMat[1][0] = (inputMat[1][2] * inputMat[2][0] - inputMat[1][0] * inputMat[2][2]) / det;
    outputMat[1][1] = (inputMat[0][0] * inputMat[2][2] - inputMat[0][2] * inputMat[2][0]) / det;
    outputMat[1][2] = (inputMat[0][2] * inputMat[1][0] - inputMat[0][0] * inputMat[1][2]) / det;

    outputMat[2][0] = (inputMat[1][0] * inputMat[2][1] - inputMat[1][1] * inputMat[2][0]) / det;
    outputMat[2][1] = (inputMat[0][1] * inputMat[2][0] - inputMat[0][0] * inputMat[2][1]) / det;
    outputMat[2][2] = (inputMat[0][0] * inputMat[1][1] - inputMat[0][1] * inputMat[1][0]) / det;
}

void ImagePerspective_Init(void)
{
    invertMatrix(change_un_Mat, invMat);
    static uint8_t BlackColor = 0;
    for (int i = 0; i < RESULT_COL; i++)
    {
        for (int j = 0; j < RESULT_ROW; j++)
        {
            int local_x = (int)((change_un_Mat[0][0] * i + change_un_Mat[0][1] * j + change_un_Mat[0][2]) / (change_un_Mat[2][0] * i + change_un_Mat[2][1] * j + change_un_Mat[2][2]));
            int local_y = (int)((change_un_Mat[1][0] * i + change_un_Mat[1][1] * j + change_un_Mat[1][2]) / (change_un_Mat[2][0] * i + change_un_Mat[2][1] * j + change_un_Mat[2][2]));
            int loc_x = (int)((invMat[0][0] * i + invMat[0][1] * j + invMat[0][2]) / (invMat[2][0] * i + invMat[2][1] * j + invMat[2][2]));
            int loc_y = (int)((invMat[1][0] * i + invMat[1][1] * j + invMat[1][2]) / (invMat[2][0] * i + invMat[2][1] * j + invMat[2][2]));
            if (loc_y < USED_ROW)
            {
                point_map[j][i][0] = loc_x;
                point_map[j][i][1] = loc_y;
            }
            if (local_x >= 0 && local_y >= 0 && local_y < USED_ROW && local_x < USED_COL)
            {
                PerImg_ip[j][i] = &PER_IMG[local_y][local_x];
            }
            else
            {
                PerImg_ip[j][i] = &BlackColor; //&PER_IMG[0][0];
            }
        }
    }
}