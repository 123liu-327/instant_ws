#ifndef FLOW_END_MATTRANSFORM_H
#define FLOW_END_MATTRANSFORM_H
#include <flow_end/follow.h>
//将图像转变为二维数组
void convertMatTo2DArray(const cv::Mat &mat, uint8_t test_img[RESULT_ROW][RESULT_COL]);
//将图像像素值翻转，加强数据
void invertImage(uint8_t img[RESULT_ROW][RESULT_COL]);
//将二维数组转变为图像
cv::Mat convert2DArrayToMat(uint8_t test_img[RESULT_ROW][RESULT_COL]);
#endif