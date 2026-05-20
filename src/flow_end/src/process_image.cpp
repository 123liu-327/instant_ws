#include <flow_end/process_image.h>

void process_image()
{
    // ԭͼ     ұ
    int local_thres = 0;
    int block_size = 7;
    int half = block_size / 2;
    int _d = 6;
    int x1 = img_raw.width / 2 - begin_x - after_bizhang_x, y1 = begin_y + after_bizhang_y;
    ipts0_num = sizeof(ipts0) / sizeof(ipts0[0]);
    int find_nums = 5;
    if (after_bizhang_x != 0)
    {
        find_nums = 10;
    }
    for (int _i = 0; _i < find_nums; _i++)
    {
        for (x1 = img_raw.width / 2 - begin_x - after_bizhang_x; x1 > 0; x1--)
        {
            for (int dy = -half; dy <= half; dy++)
            {
                local_thres += (AT_CLIP(&img_raw, x1 + _d, y1 + dy) - AT_CLIP(&img_raw, x1 - _d, y1 + dy));
            }
            local_thres /= block_size;
            if (local_thres >= thres)
                break;
            local_thres = 0;
        }
        if (local_thres >= thres)
            break;
        y1 = begin_y + after_bizhang_y - _i * 25;
    }

    if (local_thres >= thres)
        findline_lefthand_adaptive(&img_raw, block_size, clip_value, x1 - (_d), y1, ipts0, &ipts0_num);
    else
        ipts0_num = 0;
    int x2 = img_raw.width / 2 + begin_x + after_bizhang_x, y2 = begin_y + after_bizhang_y;
    ipts1_num = sizeof(ipts1) / sizeof(ipts1[0]);
    local_thres = 0;
    for (int _i = 0; _i < find_nums; _i++)
    {
        // std::cout << "_i:" << _i << " local_thres:" << local_thres << " y2:" << y2 << std::endl;
        //ROS_INFO("_i: %d local_thres: %d \n", _i, y2);
        for (x2 = img_raw.width / 2 + begin_x + after_bizhang_x; x2 < img_raw.width - 1; x2++)
        {
            for (int dy = -half; dy <= half; dy++)
            {
                local_thres -= (AT_CLIP(&img_raw, x2 + _d, y2 + dy) - AT_CLIP(&img_raw, x2 - _d, y2 + dy));
            }
            local_thres /= block_size;
            // std::cout<<"local_thres:"<<local_thres<<" x2:"<<x2<<std::endl;
            if (local_thres >= thres)
                break;
            local_thres = 0;
        }
        if (local_thres >= thres)
            break;
        y2 = begin_y + after_bizhang_y - _i * 25;
    }
    if (local_thres >= thres)
        findline_righthand_adaptive(&img_raw, block_size, clip_value, x2 + (_d), y2, ipts1, &ipts1_num);
    else
        ipts1_num = 0;

    for (int i = 0; i < ipts0_num; i++)
    {
        rpts0[i][0] = point_map[ipts0[i][1]][ipts0[i][0]][0];
        rpts0[i][1] = point_map[ipts0[i][1]][ipts0[i][0]][1];
    }
    rpts0_num = ipts0_num;

    ROS_INFO("rpts0_num: %d\n", rpts0_num);
    for (int i = 0; i < ipts1_num; i++)
    {
        rpts1[i][0] = point_map[ipts1[i][1]][ipts1[i][0]][0];
        rpts1[i][1] = point_map[ipts1[i][1]][ipts1[i][0]][1];
    }
    rpts1_num = ipts1_num;

    ROS_INFO("rpts1_num: %d\n", rpts1_num);
    //平滑处理
    blur_points(rpts0, rpts0_num, rpts0b, (int)round(line_blur_kernel));
    rpts0b_num = rpts0_num;
    blur_points(rpts1, rpts1_num, rpts1b, (int)round(line_blur_kernel));
    rpts1b_num = rpts1_num;
    //等距采样
    rpts0s_num = sizeof(rpts0s) / sizeof(rpts0s[0]);
    resample_points(rpts0b, rpts0b_num, rpts0s, &rpts0s_num, sample_dist * pixel_per_meter);
    rpts1s_num = sizeof(rpts1s) / sizeof(rpts1s[0]);
    resample_points(rpts1b, rpts1b_num, rpts1s, &rpts1s_num, sample_dist * pixel_per_meter);
    // std::cout << "rpts0s_num:" << rpts0s_num << std::endl;
    ROS_INFO("rpts0s_num: %d\n", rpts0s_num);
    // std::cout << "rpts1s_num:" << rpts1s_num << std::endl;
    ROS_INFO("rpts1s_num: %d\n", rpts1s_num);
    //局部角度估计
    local_angle_points(rpts0s, rpts0s_num, rpts0a, (int)round(angle_dist / sample_dist));
    rpts0a_num = rpts0s_num;
    local_angle_points(rpts1s, rpts1s_num, rpts1a, (int)round(angle_dist / sample_dist));
    rpts1a_num = rpts1s_num;
    //角度变化非极大值抑制
    nms_angle(rpts0a, rpts0a_num, rpts0an, (int)round(0.2 / sample_dist) * 2 + 1);
    rpts0an_num = rpts0a_num;
    nms_angle(rpts1a, rpts1a_num, rpts1an, (int)round(0.2 / sample_dist) * 2 + 1);
    rpts1an_num = rpts1a_num;

    //巡线
    track_leftline(rpts0s, rpts0s_num, rptsc0, (int)round(0.2 / sample_dist), pixel_per_meter * ROAD_WIDTH / 2+Dis_Bias_Left);
    rptsc0_num = rpts0s_num;
    track_rightline(rpts1s, rpts1s_num, rptsc1, (int)round(0.2 / sample_dist), pixel_per_meter * ROAD_WIDTH / 2+Dis_Bias_Right);
    rptsc1_num = rpts1s_num;
    // std::cout << "rptsc0_num:" << rptsc0_num << std::endl;
    ROS_INFO("rptsc0_num: %d\n", rpts1_num);
    // std::cout << "rptsc1_num:" << rptsc1_num << std::endl;
    ROS_INFO("rpts1_num: %d\n", rpts1_num);

    rptsc0e_num = sizeof(rptsc0e) / sizeof(rptsc0e[0]);
    resample_points(rptsc0, rptsc0_num, rptsc0e, &rptsc0e_num, sample_dist * pixel_per_meter);
    rptsc1e_num = sizeof(rptsc1e) / sizeof(rptsc1e[0]);
    resample_points(rptsc1, rptsc1_num, rptsc1e, &rptsc1e_num, sample_dist * pixel_per_meter);
}