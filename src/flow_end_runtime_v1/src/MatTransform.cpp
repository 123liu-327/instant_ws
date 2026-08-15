#include <flow_end/MatTransform.h>

//将图像转换为二维数组
void convertMatTo2DArray(const cv::Mat &mat, uint8_t test_img[RESULT_ROW][RESULT_COL])
{
    if (mat.channels() != 1)
    {
        ROS_INFO("Error: Input Mat must be single-channel (grayscale).\n");
        return;
    }
    if (mat.rows != RESULT_ROW || mat.cols != RESULT_COL)
    {
        ROS_INFO("Error: Input Mat size does not match the target array size.\n");
        return;
    }
    for (int i = 0; i < RESULT_ROW; ++i)
    {
        for (int j = 0; j < RESULT_COL; ++j)
        {
            test_img[i][j] = mat.at<uint8_t>(i, j);
        }
    }
}
//将图像的各个像素翻转
void invertImage(uint8_t img[RESULT_ROW][RESULT_COL])
{
    for (int y = 0; y < RESULT_ROW; ++y)
    {
        for (int x = 0; x < RESULT_COL; ++x)
        {
            img[y][x] = 255 - img[y][x]; 
        }
    }
}

//将二维数组转化为图像
cv::Mat convert2DArrayToMat(uint8_t test_img[RESULT_ROW][RESULT_COL])
{
    cv::Mat mat(RESULT_ROW, RESULT_COL, CV_8UC1);
    for (int i = 0; i < RESULT_ROW; ++i)
    {
        for (int j = 0; j < RESULT_COL; ++j)
        {
            mat.at<uint8_t>(i, j) = test_img[i][j];
        }
    }
    return mat;
}