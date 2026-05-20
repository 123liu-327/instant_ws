#include <flow_end/follow.h>
#include <flow_end/ImagePerspectiveInit.h>
#include <flow_end/MatTransform.h>
#include <flow_end/process_image.h>

#include <sensor_msgs/image_encodings.h>

// Minimal global definitions required by the image-processing pipeline.
double change_un_Mat[3][3] = {{-2.897018, 2.446196, -388.368977},
                              {-0.061836, 1.194630, -756.140464},
                              {-0.000272, 0.008324, -4.335235}};
double invMat[3][3];
int point_map[RESULT_ROW][RESULT_COL][2];
uint8_t *PerImg_ip[RESULT_ROW][RESULT_COL];
uint8_t SimBinImage[RESULT_ROW][RESULT_COL];
float mapx[RESULT_ROW][RESULT_COL];
float mapy[RESULT_ROW][RESULT_COL];
ros::Publisher pub;
ros::Publisher end_pub;

float slope = 0.0f;
float angle_deg = 0.0f;
double current_yaw_lidar = 0.0;
bool is_lidar_update = false;
std::mutex frame_mutex;
cv::Mat frame;
double current_yaw = 0.0;
double curent_wz = 0.0;
enum track_type_e track_type = TRACK_RIGHT;
bool run_car = false;
std::atomic<bool> sig_INT(false);
ros::Time move_start_time;
ros::Time move_start_time_after_laser;
int check = 0;
float Dist_1 = 100.0f;
double pre_yaw = 0.0;
double pre_angle_deg = 0.0;
bool check_after_laser = false;
bool check_imu = false;
int after_bizhang_x = 0;
int after_bizhang_y = 0;
uint8_t test_img[RESULT_ROW][RESULT_COL];

const int dir_front[4][2] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}};
const int dir_frontleft[4][2] = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};
const int dir_frontright[4][2] = {{1, -1}, {1, 1}, {-1, 1}, {-1, -1}};

image_t img_raw = DEF_IMAGE(NULL, RESULT_COL, RESULT_ROW);
float begin_x = 25;
float begin_y = 400;
int ipts0[POINTS_MAX_LEN][2];
int ipts1[POINTS_MAX_LEN][2];
int ipts0_num = 0, ipts1_num = 0;
float thres = 30;
float block_size = 7;
float clip_value = 1;
float line_blur_kernel = 7;
float pixel_per_meter = 500;
float rpts0[POINTS_MAX_LEN][2];
float rpts1[POINTS_MAX_LEN][2];
int rpts0_num = 0, rpts1_num = 0;
float rpts0b[POINTS_MAX_LEN][2];
float rpts1b[POINTS_MAX_LEN][2];
int rpts0b_num = 0, rpts1b_num = 0;
float rpts0s[POINTS_MAX_LEN][2];
float rpts1s[POINTS_MAX_LEN][2];
int rpts0s_num = 0, rpts1s_num = 0;
float sample_dist = 0.01;
float rpts0a[POINTS_MAX_LEN];
float rpts1a[POINTS_MAX_LEN];
int rpts0a_num = 0, rpts1a_num = 0;
float angle_dist = 0.1;
float rpts0an[POINTS_MAX_LEN];
float rpts1an[POINTS_MAX_LEN];
int rpts0an_num = 0, rpts1an_num = 0;
float rptsc0[POINTS_MAX_LEN][2];
float rptsc1[POINTS_MAX_LEN][2];
int rptsc0_num = 0, rptsc1_num = 0;
float rptsc0e[POINTS_MAX_LEN][2];
float rptsc1e[POINTS_MAX_LEN][2];
int rptsc0e_num = 0, rptsc1e_num = 0;

int Ypt0_rpts0s_id = 0, Ypt1_rpts1s_id = 0;
bool Ypt0_found = false, Ypt1_found = false;
int Lpt0_rpts0s_id = 0, Lpt1_rpts1s_id = 0;
bool Lpt0_found = false, Lpt1_found = false;
const float PI = 3.14159265358979323846f;
bool is_straight0 = false, is_straight1 = false;
float (*rpts)[2] = nullptr;
int rpts_num = 0;
int zeroCount = 0;
bool zero_flag = false;
uint8_t img_line_data[RESULT_ROW][RESULT_COL];
image_t img_line = DEF_IMAGE((uint8_t *)img_line_data, RESULT_COL, RESULT_ROW);
ros::Time Global_move_timer;
float Dis_Bias_Left = 0.0f;
float Dis_Bias_Right = 0.0f;
double current_linear_velocity_x = 0.0;
double current_angular_velocity_z = 0.0;
ros::Time last_imu_time;
bool imu_first_msg = true;
bool check_L_0 = false;
bool check_L_1 = false;
bool local_corner_point = false;
float odom_dist = 0.0f;
bool is_start = false;
double Time_local = 0.0;
int Round_step = 0;
float Round_step1_k = 0.0f;
float angle_rad_step1 = 0.0f;
float angle_deg_step1 = 0.0f;
ros::Time Round_timer(0);
float Laser_linear_dis = 0.0f;
bool Laser_dis_check = false;

class ImageProcessDebugNode {
public:
    ImageProcessDebugNode() : nh_("~") {
        nh_.param<std::string>("image_topic", image_topic_, "/usb_cam/image_raw");
        nh_.param<bool>("show_window", show_window_, true);
        nh_.param<bool>("publish_debug_image", publish_debug_image_, true);

        ImagePerspective_Init();

        image_sub_ = root_nh_.subscribe(image_topic_, 1, &ImageProcessDebugNode::imageCallback, this);
        if (publish_debug_image_) {
            debug_pub_ = root_nh_.advertise<sensor_msgs::Image>("/flow_end/image_process_debug", 1);
        }

        ROS_WARN("flow_end image_process_debug started. It does NOT publish /cmd_vel.");
    }

private:
    void detectCorners() {
        Ypt1_found = false;
        Lpt1_found = false;
        Ypt0_found = false;
        Lpt0_found = false;
        is_straight0 = rpts0s_num > 1.0 / sample_dist;
        is_straight1 = rpts1s_num > 1.0 / sample_dist;

        for (int i = 0; i < rpts0s_num; ++i) {
            if (rpts0an[i] == 0) continue;
            int im1 = clip(i - (int)round(angle_dist / sample_dist), 0, rpts0s_num - 1);
            int ip1 = clip(i + (int)round(angle_dist / sample_dist), 0, rpts0s_num - 1);
            float conf = fabs(rpts0a[i]) - (fabs(rpts0a[im1]) + fabs(rpts0a[ip1])) / 2;
            if (!Ypt0_found && 30.0 / 180.0 * PI < conf && conf < 65.0 / 180.0 * PI &&
                i < 0.8 / sample_dist) {
                Ypt0_rpts0s_id = i;
                Ypt0_found = true;
            }
            if (!Lpt0_found && 70.0 / 180.0 * PI < conf && conf < 110.0 / 180.0 * PI &&
                i < 0.8 / sample_dist) {
                Lpt0_rpts0s_id = i;
                Lpt0_found = true;
            }
            if (conf > 20.0 / 180.0 * PI && i < 1.0 / sample_dist) {
                is_straight0 = false;
            }
            if (Ypt0_found && Lpt0_found && !is_straight0) break;
        }

        for (int i = 0; i < rpts1s_num; ++i) {
            if (rpts1an[i] == 0) continue;
            int im1 = clip(i - (int)round(angle_dist / sample_dist), 0, rpts1s_num - 1);
            int ip1 = clip(i + (int)round(angle_dist / sample_dist), 0, rpts1s_num - 1);
            float conf = fabs(rpts1a[i]) - (fabs(rpts1a[im1]) + fabs(rpts1a[ip1])) / 2;
            if (!Ypt1_found && 30.0 / 180.0 * PI < conf && conf < 65.0 / 180.0 * PI &&
                i < 0.8 / sample_dist) {
                Ypt1_rpts1s_id = i;
                Ypt1_found = true;
            }
            if (!Lpt1_found && 70.0 / 180.0 * PI < conf && conf < 110.0 / 180.0 * PI &&
                i < 0.8 / sample_dist) {
                Lpt1_rpts1s_id = i;
                Lpt1_found = true;
            }
            if (conf > 20.0 / 180.0 * PI && i < 1.0 / sample_dist) {
                is_straight1 = false;
            }
            if (Ypt1_found && Lpt1_found && !is_straight1) break;
        }
    }

    void logDebugMetrics() {
        ROS_WARN_THROTTLE(1.0,
                          "image_process_debug ipts0=%d ipts1=%d rpts0s=%d rpts1s=%d "
                          "rptsc0e=%d rptsc1e=%d L0=%d L1=%d Y0=%d Y1=%d",
                          ipts0_num, ipts1_num, rpts0s_num, rpts1s_num,
                          rptsc0e_num, rptsc1e_num,
                          Lpt0_found, Lpt1_found, Ypt0_found, Ypt1_found);
    }

    void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (const cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat resized;
        cv::resize(cv_ptr->image, resized, cv::Size(RESULT_COL, RESULT_ROW));

        cv::Mat gray;
        cv::cvtColor(resized, gray, cv::COLOR_BGR2GRAY);
        convertMatTo2DArray(gray, PER_IMG);
        invertImage(PER_IMG);
        img_raw.data = PER_IMG[0];
        process_image();
        detectCorners();
        logDebugMetrics();

        invertImage(PER_IMG);
        for (int i = 0; i < RESULT_ROW; ++i) {
            for (int j = 0; j < RESULT_COL; ++j) {
                img_line_data[i][j] = ImageUsed[i][j];
            }
        }

        for (int i = 0; i < rptsc0e_num; ++i) {
            AT_IMAGE(&img_line, clip(rptsc0e[i][0], 0, img_line.width - 1),
                     clip(rptsc0e[i][1], 0, img_line.height - 1)) = 0;
        }
        for (int i = 0; i < rptsc1e_num; ++i) {
            AT_IMAGE(&img_line, clip(rptsc1e[i][0], 0, img_line.width - 1),
                     clip(rptsc1e[i][1], 0, img_line.height - 1)) = 80;
        }
        for (int i = 0; i < rpts0s_num; ++i) {
            AT_IMAGE(&img_line, clip(rpts0s[i][0], 0, img_line.width - 1),
                     clip(rpts0s[i][1], 0, img_line.height - 1)) = 0;
        }
        for (int i = 0; i < rpts1s_num; ++i) {
            AT_IMAGE(&img_line, clip(rpts1s[i][0], 0, img_line.width - 1),
                     clip(rpts1s[i][1], 0, img_line.height - 1)) = 80;
        }

        cv::Mat debug_gray = convert2DArrayToMat(img_line_data);
        if (show_window_) {
            cv::imshow("flow_end image process debug", debug_gray);
            cv::waitKey(1);
        }

        if (publish_debug_image_) {
            sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(
                msg->header,
                sensor_msgs::image_encodings::MONO8,
                debug_gray).toImageMsg();
            debug_pub_.publish(out_msg);
        }
    }

    ros::NodeHandle root_nh_;
    ros::NodeHandle nh_;
    ros::Subscriber image_sub_;
    ros::Publisher debug_pub_;
    std::string image_topic_;
    bool show_window_ = true;
    bool publish_debug_image_ = true;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "flow_end_image_process_debug");
    ImageProcessDebugNode node;
    ros::spin();
    cv::destroyAllWindows();
    return 0;
}
