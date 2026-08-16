#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <zbar.h>
#include <std_msgs/String.h>
#include<iostream>
std_msgs::String startInfo;
class QRCodeScanner {
private:
    ros::NodeHandle nh_;

    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Publisher qr_pub_;
    ros::Subscriber start_sub;
    int isEnd;
public:
    QRCodeScanner() : it_(nh_) {
        // 订阅摄像头话题
      //  image_sub_ = it_.subscribe("/ucar_camera/image_raw", 1, &QRCodeScanner::imageCallback, this);
        
        // 发布二维码结果
        qr_pub_ = nh_.advertise<std_msgs::String>("/qr_code_result", 10);
        start_sub = nh_.subscribe<std_msgs::String>("/qr_node_start",1,&QRCodeScanner::startCallback,this);    
        ros::Rate r1(100);
        ros::Rate r(10);
        while(ros::ok())
        {
            r.sleep();
            if(startInfo.data=="start!")
            {
                ROS_INFO("qr_node begin scan!");
                break;
            }   
            ros::spinOnce();
        }
        image_sub_ = it_.subscribe("/ucar_camera/image_raw", 1, &QRCodeScanner::imageCallback, this);
        ROS_INFO("QR Code Scanner initialized");
        for(int i=0;i<5;i++)
        {
            ros::spinOnce();
            r1.sleep();
        }
        
    }
    
    ~QRCodeScanner() {
        cv::destroyAllWindows();
    }
    void startCallback(const std_msgs::StringConstPtr& msg)
    {
        ROS_INFO("qr_node get startinfo:%s",msg->data.c_str());
        startInfo.data=msg->data;
    }
    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        
        // 转换为灰度图像用于二维码扫描
        cv::Mat gray;
        cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);
        
        // 创建ZBar扫描器
    // 配置 ZBar 扫描器
        zbar::ImageScanner scanner;
        scanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);  // 明确启用 QR Code
        
        // 包装图像数据
        zbar::Image zbar_image(
            gray.cols, 
            gray.rows, 
            "Y800", 
            gray.data, 
            gray.cols * gray.rows * gray.elemSize()
        );
        
        // 扫描图像中的二维码
        int n = scanner.scan(zbar_image);
        
        if (n > 0) {
            for (zbar::Image::SymbolIterator symbol = zbar_image.symbol_begin(); 
                 symbol != zbar_image.symbol_end(); ++symbol) {
                
                // 获取二维码数据
                std::string qr_data = symbol->get_data();
                ROS_INFO("QR Code detected: %s", qr_data.c_str());
                
                // 发布结果
                std_msgs::String result_msg;
                result_msg.data = qr_data;
                std::cout<<"result str="<<result_msg.data<<'\n';
                
                qr_pub_.publish(result_msg);
                
                // 绘制二维码边界
                std::vector<cv::Point> points;
                for (int i = 0; i < symbol->get_location_size(); i++) {
                    points.push_back(cv::Point(symbol->get_location_x(i), 
                                              symbol->get_location_y(i)));
                }
                
                // 绘制边界
                std::vector<std::vector<cv::Point>> contours;
                contours.push_back(points);
                cv::drawContours(cv_ptr->image, contours, -1, cv::Scalar(0, 255, 0), 2);
                
                // 绘制二维码内容文本
                cv::putText(cv_ptr->image, qr_data, cv::Point(10, 30), 
                            cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
            }
        }
        
        // 显示结果（可选）
        cv::imshow("QR Code Scanner", cv_ptr->image);
        cv::waitKey(1);
    }
};

int main(int argc,char *argv[])
{
    ros::init(argc,argv, "qr_code_scanner");
    QRCodeScanner qr_scanner;
    ros::spin();
    return 0;
}