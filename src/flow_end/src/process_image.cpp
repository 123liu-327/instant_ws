#include <flow_end/process_image.h>

#include <cmath>

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

ForwardCrossbarResult forward_crossbar_result = {
    false,
    0,
    0,
    0,
    0.0f,
    0.0f,
    0.0f,
    0.0f
};


namespace {

float localMeanAt(image_t *img, int x, int y, int half_window)
{
    // Local background estimate, matching the idea used by findline_adaptive:
    // do not classify a lane pixel by a fixed gray value; compare it with its
    // nearby floor/background instead.
    float sum = 0.0f;
    int count = 0;
    for (int dy = -half_window; dy <= half_window; ++dy) {
        for (int dx = -half_window; dx <= half_window; ++dx) {
            sum += AT_CLIP(img, x + dx, y + dy);
            count++;
        }
    }
    return count > 0 ? sum / count : 0.0f;
}

bool isAdaptiveDarkLinePixel(image_t *img, int x, int y,
                             int half_window, float clip_value,
                             float *local_mean_out = nullptr)
{
    // Original white lane markings are inverted before process_image(), so
    // lane/crossbar pixels become darker than the surrounding gray floor.
    // A pixel is accepted only if it is darker than the local mean by
    // clip_value, which avoids treating the whole gray floor as a line.
    const float local_mean = localMeanAt(img, x, y, half_window);
    if (local_mean_out) {
        *local_mean_out = local_mean;
    }
    return AT_CLIP(img, x, y) < local_mean - clip_value;
}

}  // namespace

bool detect_forward_crossbar()
{
    forward_crossbar_result = {
        false,
        0,
        0,
        0,
        0.0f,
        0.0f,
        0.0f,
        0.0f
    };

    if (img_raw.data == nullptr || pixel_per_meter <= 1.0f) {
        ROS_WARN_THROTTLE(1.0,
                          "[Y_CROSSBAR_DEBUG] invalid_input | img=%d | ppm=%.1f",
                          img_raw.data != nullptr,
                          pixel_per_meter);
        return false;
    }

    const int image_center_x = RESULT_COL / 2;
    const int roi_half_width = 160;
    const int roi_x_min = clip(image_center_x - roi_half_width, 0, RESULT_COL - 1);
    const int roi_x_max = clip(image_center_x + roi_half_width, 0, RESULT_COL - 1);
    const int roi_y_min = clip(static_cast<int>(RESULT_ROW * 0.35f), 0, RESULT_ROW - 1);
    const int roi_y_max = clip(static_cast<int>(RESULT_ROW * 0.75f), 0, RESULT_ROW - 1);
    const int roi_width = roi_x_max - roi_x_min + 1;

    const int adaptive_block_size = 9;
    const int adaptive_half_window = adaptive_block_size / 2;
    const float crossbar_clip_value = 12.0f;
    const float max_width_ratio = 0.85f;

    const int min_width_px = std::max(8, static_cast<int>(std::round(0.25f * pixel_per_meter)));
    const int max_width_px = std::max(min_width_px, static_cast<int>(std::round(roi_width * max_width_ratio)));
    const int center_tolerance_px = std::max(4, static_cast<int>(std::round(0.12f * pixel_per_meter)));

    int best_left = 0;
    int best_right = 0;
    int best_y = 0;
    int best_width = 0;
    int best_center_error = RESULT_COL;
    float best_local_mean = 0.0f;
    int best_pixel = 0;

    int longest_left = 0;
    int longest_right = 0;
    int longest_y = 0;
    int longest_width = 0;
    int longest_center_error = RESULT_COL;
    float longest_local_mean = 0.0f;
    int longest_pixel = 0;
    int line_pixel_count = 0;

    for (int y = roi_y_min; y <= roi_y_max; ++y) {
        int run_start = -1;
        float run_mean_sum = 0.0f;
        int run_pixel_sum = 0;
        int run_count = 0;

        for (int x = roi_x_min; x <= roi_x_max + 1; ++x) {
            const bool in_roi = x <= roi_x_max;
            float local_mean = 0.0f;
            // This is the actual adaptive lane-pixel test for the crossbar:
            // scan each row, but use local contrast instead of ImageUsed < 128.
            const bool is_line_pixel =
                in_roi && isAdaptiveDarkLinePixel(&img_raw, x, y,
                                                   adaptive_half_window,
                                                   crossbar_clip_value,
                                                   &local_mean);

            if (is_line_pixel) {
                if (run_start < 0) {
                    run_start = x;
                    run_mean_sum = 0.0f;
                    run_pixel_sum = 0;
                    run_count = 0;
                }
                line_pixel_count++;
                run_mean_sum += local_mean;
                run_pixel_sum += AT_CLIP(&img_raw, x, y);
                run_count++;
            }

            if ((!is_line_pixel || !in_roi) && run_start >= 0) {
                const int run_end = x - 1;
                const int width = run_end - run_start + 1;
                const int center_x = (run_start + run_end) / 2;
                const int center_error = std::abs(center_x - image_center_x);
                const float avg_local_mean = run_count > 0 ? run_mean_sum / run_count : 0.0f;
                const int avg_pixel = run_count > 0 ? static_cast<int>(std::round(static_cast<float>(run_pixel_sum) / run_count)) : 0;

                if (width > longest_width ||
                    (width == longest_width && center_error < longest_center_error)) {
                    longest_left = run_start;
                    longest_right = run_end;
                    longest_y = y;
                    longest_width = width;
                    longest_center_error = center_error;
                    longest_local_mean = avg_local_mean;
                    longest_pixel = avg_pixel;
                }

                const bool width_ok = width >= min_width_px && width <= max_width_px;
                const bool center_ok = center_error <= center_tolerance_px;
                if (width_ok && center_ok) {
                    const bool better_width = width > best_width;
                    const bool same_width_better_center =
                        width == best_width && center_error < best_center_error;
                    if (better_width || same_width_better_center) {
                        best_left = run_start;
                        best_right = run_end;
                        best_y = y;
                        best_width = width;
                        best_center_error = center_error;
                        best_local_mean = avg_local_mean;
                        best_pixel = avg_pixel;
                    }
                }

                run_start = -1;
                run_mean_sum = 0.0f;
                run_pixel_sum = 0;
                run_count = 0;
            }
        }
    }

    if (best_width <= 0) {
        const int longest_center_x = (longest_left + longest_right) / 2;
        ROS_WARN_THROTTLE(0.5,
                          "[Y_CROSSBAR_DEBUG] adaptive_not_found | line_px=%d | longest_width=%d/%d~%d | longest_center=(%d,%d) | center_error=%d/%d | local_mean=%.1f | pixel=%d | clip=%.1f | block=%d",
                          line_pixel_count,
                          longest_width, min_width_px, max_width_px,
                          longest_center_x, longest_y,
                          longest_center_error, center_tolerance_px,
                          longest_local_mean,
                          longest_pixel,
                          crossbar_clip_value,
                          adaptive_block_size);
        return false;
    }

    const int center_x = (best_left + best_right) / 2;
    const int center_y = best_y;
    const float map_x = static_cast<float>(point_map[center_y][center_x][0]);
    const float map_y = static_cast<float>(point_map[center_y][center_x][1]);
    const float ref_x = RESULT_COL / 2.0f;
    const float ref_y = RESULT_ROW + 10.0f;

    forward_crossbar_result.found = true;
    forward_crossbar_result.center_x = center_x;
    forward_crossbar_result.center_y = center_y;
    forward_crossbar_result.width_px = best_width;
    forward_crossbar_result.map_x = map_x;
    forward_crossbar_result.map_y = map_y;
    forward_crossbar_result.long_m = -(map_y - ref_y) / pixel_per_meter;
    forward_crossbar_result.lat_m = -(map_x - ref_x) / pixel_per_meter;

    ROS_WARN_THROTTLE(0.5,
                      "[Y_CROSSBAR_DEBUG] adaptive_found | center=(%d,%d) | width=%d/%d~%d | center_error=%d/%d | map=(%.1f,%.1f) | long=%.3fm | lat=%.3fm | local_mean=%.1f | pixel=%d | line_px=%d | clip=%.1f | block=%d",
                      center_x, center_y,
                      best_width, min_width_px, max_width_px,
                      best_center_error, center_tolerance_px,
                      forward_crossbar_result.map_x,
                      forward_crossbar_result.map_y,
                      forward_crossbar_result.long_m,
                      forward_crossbar_result.lat_m,
                      best_local_mean,
                      best_pixel,
                      line_pixel_count,
                      crossbar_clip_value,
                      adaptive_block_size);

    return true;
}
