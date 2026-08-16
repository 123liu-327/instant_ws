#include <algorithm>
#include <chrono>
#include <set>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <zbar.h>

#include <ucar_qr_decoder/ZBarDecodeRequest.h>
#include <ucar_qr_decoder/ZBarDecodeResult.h>

namespace {

std::vector<std::string> scanQr(zbar::ImageScanner& scanner, const cv::Mat& input) {
  cv::Mat gray;
  if (input.channels() == 1) {
    gray = input;
  } else {
    cv::cvtColor(input, gray, cv::COLOR_BGR2GRAY);
  }
  if (!gray.isContinuous()) {
    gray = gray.clone();
  }

  zbar::Image image(gray.cols, gray.rows, "Y800", gray.data,
                    static_cast<unsigned long>(gray.total()));
  scanner.scan(image);
  std::set<std::string> unique;
  for (zbar::Image::SymbolIterator symbol = image.symbol_begin();
       symbol != image.symbol_end(); ++symbol) {
    if (!symbol->get_data().empty()) {
      unique.insert(symbol->get_data());
    }
  }
  image.set_data(nullptr, 0);
  return std::vector<std::string>(unique.begin(), unique.end());
}

class ZBarDecoderNode {
 public:
  ZBarDecoderNode() : private_nh_("~") {
    private_nh_.param<std::string>("request_topic", request_topic_,
                                   "/qr_decoder/internal/zbar_request");
    private_nh_.param<std::string>("result_topic", result_topic_,
                                   "/qr_decoder/internal/zbar_result");
    private_nh_.param<double>("clahe_clip_limit", clahe_clip_limit_, 2.0);
    private_nh_.param<double>("sharpen_sigma", sharpen_sigma_, 1.0);
    scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 0);
    scanner_.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);
    clahe_ = cv::createCLAHE(clahe_clip_limit_, cv::Size(8, 8));

    result_pub_ = nh_.advertise<ucar_qr_decoder::ZBarDecodeResult>(result_topic_, 1);
    request_sub_ = nh_.subscribe(request_topic_, 1, &ZBarDecoderNode::requestCallback, this);
    ROS_INFO("ZBar QR backend ready: %s -> %s", request_topic_.c_str(), result_topic_.c_str());
  }

 private:
  void requestCallback(const ucar_qr_decoder::ZBarDecodeRequest::ConstPtr& request) {
    const auto started = std::chrono::steady_clock::now();
    ucar_qr_decoder::ZBarDecodeResult result;
    result.job_id = request->job_id;
    result.stage = "zbar_none";
    try {
      const cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(
          request->image, request, sensor_msgs::image_encodings::BGR8);
      if (request->mode == ucar_qr_decoder::ZBarDecodeRequest::MODE_RAW_ONLY) {
        result.decoded = scanQr(scanner_, cv_ptr->image);
        if (!result.decoded.empty()) {
          result.stage = "zbar_raw";
        }
      } else {
        cv::Mat gray;
        cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);
        cv::Mat enhanced;
        clahe_->apply(gray, enhanced);
        cv::Mat blurred;
        cv::GaussianBlur(enhanced, blurred, cv::Size(0, 0), sharpen_sigma_);
        cv::Mat sharpened;
        cv::addWeighted(enhanced, 1.6, blurred, -0.6, 0.0, sharpened);
        const std::vector<double> scales{1.0, 1.5, 2.0};
        for (const double scale : scales) {
          cv::Mat candidate;
          if (scale == 1.0) {
            candidate = sharpened;
          } else {
            cv::resize(sharpened, candidate, cv::Size(), scale, scale, cv::INTER_CUBIC);
          }
          result.decoded = scanQr(scanner_, candidate);
          if (!result.decoded.empty()) {
            result.stage = "zbar_enhanced_" + std::to_string(scale);
            break;
          }
        }
      }
    } catch (const cv_bridge::Exception& error) {
      result.stage = std::string("zbar_cv_bridge_error:") + error.what();
      ROS_WARN("ZBar request %lu image conversion failed: %s",
               static_cast<unsigned long>(request->job_id), error.what());
    } catch (const std::exception& error) {
      result.stage = std::string("zbar_error:") + error.what();
      ROS_WARN("ZBar request %lu failed: %s",
               static_cast<unsigned long>(request->job_id), error.what());
    }
    result.decode_ms = static_cast<float>(
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started)
            .count());
    result_pub_.publish(result);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber request_sub_;
  ros::Publisher result_pub_;
  std::string request_topic_;
  std::string result_topic_;
  double clahe_clip_limit_;
  double sharpen_sigma_;
  zbar::ImageScanner scanner_;
  cv::Ptr<cv::CLAHE> clahe_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "qr_zbar_backend");
  ZBarDecoderNode node;
  ros::spin();
  return 0;
}
