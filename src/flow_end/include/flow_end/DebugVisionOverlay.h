#ifndef FLOW_END_DEBUG_VISION_OVERLAY_H
#define FLOW_END_DEBUG_VISION_OVERLAY_H

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace flow_end {
namespace debug_overlay {

struct CornerCandidate {
    bool valid = false;
    int index = -1;
    double confidence_deg = 0.0;
};

struct LanePixelStats {
    int count = 0;
    int raw_min = 255;
    int raw_max = 0;
    double raw_sum = 0.0;
    int proc_min = 255;
    int proc_max = 0;
    double proc_sum = 0.0;
};

inline cv::Point clippedPoint(const float point[2], const cv::Size &size) {
    return cv::Point(
        std::max(0, std::min(size.width - 1, static_cast<int>(std::lround(point[0])))),
        std::max(0, std::min(size.height - 1, static_cast<int>(std::lround(point[1])))));
}

inline double cornerConfidenceDeg(const float *angles, int point_count, int index,
                                  double angle_dist, double sample_dist) {
    if (angles == nullptr || point_count <= 0 || index < 0 || index >= point_count ||
        sample_dist <= 0.0) {
        return 0.0;
    }
    const int offset = std::max(1, static_cast<int>(std::lround(angle_dist / sample_dist)));
    const int before = std::max(0, index - offset);
    const int after = std::min(point_count - 1, index + offset);
    const double confidence = std::abs(angles[index]) -
        (std::abs(angles[before]) + std::abs(angles[after])) / 2.0;
    return confidence * 180.0 / CV_PI;
}

inline CornerCandidate strongestCandidate(const float *angles, const float *nms_angles,
                                          int point_count, double angle_dist,
                                          double sample_dist) {
    CornerCandidate best;
    for (int i = 0; i < point_count; ++i) {
        if (nms_angles != nullptr && nms_angles[i] == 0.0f) {
            continue;
        }
        const double confidence = cornerConfidenceDeg(
            angles, point_count, i, angle_dist, sample_dist);
        if (!best.valid || confidence > best.confidence_deg) {
            best.valid = true;
            best.index = i;
            best.confidence_deg = confidence;
        }
    }
    return best;
}

inline void drawPointSeries(cv::Mat &image, const float points[][2], int point_count,
                            const cv::Scalar &color, int radius = 1, int stride = 1) {
    if (points == nullptr || point_count <= 0) {
        return;
    }
    stride = std::max(1, stride);
    for (int i = 0; i < point_count; i += stride) {
        cv::circle(image, clippedPoint(points[i], image.size()), radius, color, -1,
                   cv::LINE_AA);
    }
}

inline LanePixelStats sampleLanePixels(const cv::Mat &raw_gray,
                                       const float points[][2],
                                       int point_count,
                                       int stride = 1) {
    LanePixelStats stats;
    if (raw_gray.empty() || raw_gray.channels() != 1 || points == nullptr ||
        point_count <= 0) {
        return stats;
    }

    stride = std::max(1, stride);
    for (int i = 0; i < point_count; i += stride) {
        const cv::Point point = clippedPoint(points[i], raw_gray.size());
        const int raw_value =
            static_cast<int>(raw_gray.at<unsigned char>(point.y, point.x));
        const int proc_value = 255 - raw_value;
        stats.count++;
        stats.raw_min = std::min(stats.raw_min, raw_value);
        stats.raw_max = std::max(stats.raw_max, raw_value);
        stats.raw_sum += raw_value;
        stats.proc_min = std::min(stats.proc_min, proc_value);
        stats.proc_max = std::max(stats.proc_max, proc_value);
        stats.proc_sum += proc_value;
    }
    return stats;
}

inline std::string pixelStatsText(const std::string &name,
                                  const LanePixelStats &stats) {
    std::ostringstream text;
    text << name << " n=" << stats.count;
    if (stats.count <= 0) {
        text << " none";
        return text.str();
    }

    text << " raw=" << stats.raw_min << "/"
         << std::fixed << std::setprecision(1)
         << (stats.raw_sum / stats.count) << "/"
         << stats.raw_max
         << " proc=" << stats.proc_min << "/"
         << (stats.proc_sum / stats.count) << "/"
         << stats.proc_max;
    return text.str();
}

inline std::string lanePixelStatsLine(const cv::Mat &raw_gray,
                                      const float left_points[][2],
                                      int left_count,
                                      const float right_points[][2],
                                      int right_count,
                                      int stride = 2) {
    return "pixel " +
        pixelStatsText("L", sampleLanePixels(raw_gray, left_points, left_count, stride)) +
        " | " +
        pixelStatsText("R", sampleLanePixels(raw_gray, right_points, right_count, stride));
}

inline void drawCornerMarker(cv::Mat &image, const float points[][2], int point_count,
                             int index, const std::string &name,
                             const cv::Scalar &color, bool cross,
                             double confidence_deg) {
    if (points == nullptr || index < 0 || index >= point_count) {
        return;
    }
    const cv::Point point = clippedPoint(points[index], image.size());
    if (cross) {
        cv::drawMarker(image, point, color, cv::MARKER_TILTED_CROSS, 18, 3, cv::LINE_AA);
    } else {
        cv::circle(image, point, 9, color, 3, cv::LINE_AA);
        cv::circle(image, point, 2, color, -1, cv::LINE_AA);
    }

    std::ostringstream label;
    label << name << " id=" << index << " (" << point.x << "," << point.y << ")"
          << " c=" << std::fixed << std::setprecision(1) << confidence_deg << "deg";
    const int label_x = std::max(2, std::min(image.cols - 250, point.x + 12));
    const int label_y = std::max(16, std::min(image.rows - 4, point.y - 10));
    cv::putText(image, label.str(), cv::Point(label_x, label_y),
                cv::FONT_HERSHEY_SIMPLEX, 0.43, cv::Scalar(0, 0, 0), 3, cv::LINE_AA);
    cv::putText(image, label.str(), cv::Point(label_x, label_y),
                cv::FONT_HERSHEY_SIMPLEX, 0.43, color, 1, cv::LINE_AA);
}

inline void drawCandidateMarker(cv::Mat &image, const float points[][2], int point_count,
                                const CornerCandidate &candidate, const std::string &name,
                                const cv::Scalar &color) {
    if (!candidate.valid || points == nullptr || candidate.index < 0 ||
        candidate.index >= point_count) {
        return;
    }
    const cv::Point point = clippedPoint(points[candidate.index], image.size());
    cv::drawMarker(image, point, color, cv::MARKER_DIAMOND, 12, 1, cv::LINE_AA);
    std::ostringstream label;
    label << name << " id=" << candidate.index << " c=" << std::fixed
          << std::setprecision(1) << candidate.confidence_deg << "deg";
    cv::putText(image, label.str(),
                cv::Point(std::max(2, std::min(image.cols - 170, point.x + 8)),
                          std::max(14, std::min(image.rows - 3, point.y + 16))),
                cv::FONT_HERSHEY_SIMPLEX, 0.36, color, 1, cv::LINE_AA);
}

inline void drawStatusPanel(cv::Mat &image, const std::vector<std::string> &lines) {
    if (lines.empty()) {
        return;
    }
    constexpr double font_scale = 0.40;
    constexpr int thickness = 1;
    const int available_width = std::max(40, image.cols - 16);
    std::vector<std::string> wrapped_lines;
    for (const std::string &line : lines) {
        std::istringstream words(line);
        std::string word;
        std::string current;
        while (words >> word) {
            const std::string candidate = current.empty() ? word : current + " " + word;
            int baseline = 0;
            const int width = cv::getTextSize(candidate, cv::FONT_HERSHEY_SIMPLEX,
                                              font_scale, thickness, &baseline).width;
            if (!current.empty() && width > available_width) {
                wrapped_lines.push_back(current);
                current = word;
            } else {
                current = candidate;
            }
        }
        wrapped_lines.push_back(current);
    }
    const int panel_height = std::min(
        image.rows, 10 + static_cast<int>(wrapped_lines.size()) * 16);
    cv::Mat shaded = image.clone();
    cv::rectangle(shaded, cv::Rect(0, 0, image.cols, panel_height),
                  cv::Scalar(12, 12, 12), cv::FILLED);
    cv::addWeighted(shaded, 0.72, image, 0.28, 0.0, image);
    for (std::size_t i = 0; i < wrapped_lines.size(); ++i) {
        const int y = 16 + static_cast<int>(i) * 16;
        if (y >= image.rows) {
            break;
        }
        cv::putText(image, wrapped_lines[i], cv::Point(8, y),
                    cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(245, 245, 245),
                    thickness, cv::LINE_AA);
    }
}

inline std::string cornerStatus(const std::string &name, bool found, int index,
                                const float points[][2], int point_count,
                                const float *angles, double angle_dist,
                                double sample_dist) {
    std::ostringstream text;
    text << name << "=" << (found ? "FOUND" : "MISS");
    if (found && points != nullptr && index >= 0 && index < point_count) {
        const double confidence = cornerConfidenceDeg(
            angles, point_count, index, angle_dist, sample_dist);
        text << " id=" << index << " xy=" << static_cast<int>(std::lround(points[index][0]))
             << "," << static_cast<int>(std::lround(points[index][1]))
             << " c=" << std::fixed << std::setprecision(1) << confidence << "deg";
    }
    return text.str();
}

}  // namespace debug_overlay
}  // namespace flow_end

#endif
