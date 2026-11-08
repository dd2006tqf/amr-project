#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

namespace
{
constexpr double kPi = 3.14159265358979323846;

struct ColorDetection
{
  std::string name;
  cv::Rect box;
  double area;
};

struct ShapeDetection
{
  std::string name;
  cv::Rect box;
  double area;
};

struct TemplateDetection
{
  bool found{false};
  double score{0.0};
  cv::Point location{0, 0};
  cv::Size size{0, 0};
};
}  // namespace

class OpenCvDemoNode final : public rclcpp::Node
{
public:
  OpenCvDemoNode()
  : Node("opencv_demo_node")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "");
    test_image_topic_ = declare_parameter<std::string>(
      "test_image_topic", "/opencv/image_test");
    annotated_topic_ = declare_parameter<std::string>(
      "annotated_topic", "/opencv/image_annotated");
    mask_topic_ = declare_parameter<std::string>(
      "mask_topic", "/opencv/color_mask");
    detections_topic_ = declare_parameter<std::string>(
      "detections_topic", "/opencv/detections");
    template_path_ = declare_parameter<std::string>("template_path", "");
    use_test_image_ = declare_parameter<bool>("use_test_image", true);
    publish_rate_ = declare_parameter<double>("publish_rate", 5.0);
    match_threshold_ = declare_parameter<double>("match_threshold", 0.80);
    min_contour_area_ = declare_parameter<double>("min_contour_area", 150.0);

    if (!template_path_.empty()) {
      template_image_ = cv::imread(template_path_, cv::IMREAD_COLOR);
      if (template_image_.empty()) {
        RCLCPP_WARN(
          get_logger(), "Unable to load template_path '%s'; using built-in template",
          template_path_.c_str());
      }
    }
    if (template_image_.empty()) {
      template_image_ = makeTemplatePatch();
    }

    const auto qos = rclcpp::SensorDataQoS();
    test_image_pub_ = create_publisher<sensor_msgs::msg::Image>(test_image_topic_, qos);
    annotated_pub_ = create_publisher<sensor_msgs::msg::Image>(annotated_topic_, qos);
    mask_pub_ = create_publisher<sensor_msgs::msg::Image>(mask_topic_, qos);
    detections_pub_ = create_publisher<std_msgs::msg::String>(detections_topic_, 10);

    if (!input_topic_.empty()) {
      input_sub_ = create_subscription<sensor_msgs::msg::Image>(
        input_topic_, qos,
        std::bind(&OpenCvDemoNode::imageCallback, this, std::placeholders::_1));
      RCLCPP_INFO(get_logger(), "Subscribing to input image: %s", input_topic_.c_str());
    }

    if (use_test_image_) {
      const double safe_rate = std::max(0.2, publish_rate_);
      test_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::duration<double>(1.0 / safe_rate)),
        std::bind(&OpenCvDemoNode::publishTestImage, this));
      RCLCPP_INFO(
        get_logger(), "Publishing synthetic test images on %s at %.2f Hz",
        test_image_topic_.c_str(), safe_rate);
    }

    if (input_topic_.empty() && !use_test_image_) {
      RCLCPP_WARN(get_logger(), "Both input_topic is empty and use_test_image is false");
    }
  }

private:
  static cv::Mat makeTemplatePatch()
  {
    cv::Mat patch(72, 72, CV_8UC3, cv::Scalar(28, 28, 28));
    cv::rectangle(patch, cv::Rect(3, 3, 66, 66), cv::Scalar(0, 220, 255), 3);
    cv::line(patch, cv::Point(16, 36), cv::Point(56, 36), cv::Scalar(255, 255, 255), 3);
    cv::line(patch, cv::Point(36, 16), cv::Point(36, 56), cv::Scalar(255, 255, 255), 3);
    cv::circle(patch, cv::Point(36, 36), 7, cv::Scalar(0, 80, 255), -1);
    return patch;
  }

  cv::Mat makeSyntheticImage() const
  {
    cv::Mat image(480, 640, CV_8UC3, cv::Scalar(34, 34, 34));

    cv::circle(image, cv::Point(145, 135), 55, cv::Scalar(0, 0, 230), -1);
    cv::circle(image, cv::Point(145, 135), 55, cv::Scalar(255, 255, 255), 3);

    cv::rectangle(image, cv::Rect(270, 80, 150, 105), cv::Scalar(0, 190, 0), -1);
    cv::rectangle(image, cv::Rect(270, 80, 150, 105), cv::Scalar(255, 255, 255), 3);

    std::vector<cv::Point> triangle{
      cv::Point(125, 330), cv::Point(225, 330), cv::Point(175, 235)};
    cv::fillConvexPoly(image, triangle, cv::Scalar(230, 80, 0));
    cv::polylines(image, triangle, true, cv::Scalar(255, 255, 255), 3);

    cv::circle(image, cv::Point(490, 165), 45, cv::Scalar(230, 80, 0), -1);
    cv::circle(image, cv::Point(490, 165), 45, cv::Scalar(255, 255, 255), 3);

    if (template_image_.cols <= image.cols && template_image_.rows <= image.rows) {
      const cv::Rect roi(500, 330, template_image_.cols, template_image_.rows);
      template_image_.copyTo(image(roi));
    }
    cv::putText(
      image, "OpenCV ROS2 TEST", cv::Point(20, 460),
      cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(220, 220, 220), 2, cv::LINE_AA);
    return image;
  }

  void publishTestImage()
  {
    const auto stamp = now();
    std_msgs::msg::Header header;
    header.stamp = stamp;
    header.frame_id = "opencv_test";
    const cv::Mat image = makeSyntheticImage();

    auto message = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
    test_image_pub_->publish(*message);
    processImage(image, header);
  }

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr message)
  {
    try {
      const auto cv_image = cv_bridge::toCvCopy(
        message, sensor_msgs::image_encodings::BGR8);
      processImage(cv_image->image, message->header);
    } catch (const cv_bridge::Exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Image conversion failed: %s", error.what());
    }
  }

  std::vector<ColorDetection> detectColors(
    const cv::Mat & bgr, cv::Mat & combined_mask) const
  {
    cv::Mat hsv;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
    combined_mask = cv::Mat::zeros(bgr.size(), CV_8UC1);

    const std::vector<std::tuple<std::string, cv::Scalar, cv::Scalar>> ranges{
      {"red", cv::Scalar(0, 90, 60), cv::Scalar(10, 255, 255)},
      {"green", cv::Scalar(35, 70, 50), cv::Scalar(85, 255, 255)},
      {"blue", cv::Scalar(90, 70, 50), cv::Scalar(135, 255, 255)}
    };

    std::vector<ColorDetection> detections;
    for (const auto & range : ranges) {
      cv::Mat mask;
      cv::inRange(hsv, std::get<1>(range), std::get<2>(range), mask);
      if (std::get<0>(range) == "red") {
        cv::Mat red_wrap;
        cv::inRange(
          hsv, cv::Scalar(170, 90, 60), cv::Scalar(179, 255, 255), red_wrap);
        cv::bitwise_or(mask, red_wrap, mask);
      }
      const cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_ELLIPSE, cv::Size(5, 5));
      cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
      cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
      cv::bitwise_or(combined_mask, mask, combined_mask);

      std::vector<std::vector<cv::Point>> contours;
      cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
      for (const auto & contour : contours) {
        const double area = cv::contourArea(contour);
        if (area < min_contour_area_) {
          continue;
        }
        detections.push_back(
          {std::get<0>(range), cv::boundingRect(contour), area});
      }
    }
    return detections;
  }

  std::vector<ShapeDetection> detectShapes(const cv::Mat & bgr) const
  {
    cv::Mat gray;
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0.0);
    cv::Mat edges;
    cv::Canny(gray, edges, 70.0, 150.0);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    std::vector<ShapeDetection> detections;
    for (const auto & contour : contours) {
      const double area = cv::contourArea(contour);
      const double perimeter = cv::arcLength(contour, true);
      if (area < min_contour_area_ || perimeter <= 0.0) {
        continue;
      }
      std::vector<cv::Point> polygon;
      cv::approxPolyDP(contour, polygon, 0.02 * perimeter, true);
      const double circularity = 4.0 * kPi * area / (perimeter * perimeter);
      std::string name;
      if (polygon.size() == 3) {
        name = "triangle";
      } else if (polygon.size() == 4) {
        name = "rectangle";
      } else if (circularity > 0.70) {
        name = "circle";
      } else {
        continue;
      }
      const cv::Rect box = cv::boundingRect(contour);
      bool duplicate = false;
      for (auto & existing : detections) {
        if (existing.name != name) {
          continue;
        }
        const cv::Rect intersection = existing.box & box;
        const double min_box_area = static_cast<double>(
          std::min(existing.box.area(), box.area()));
        const double overlap = min_box_area > 0.0 ?
          static_cast<double>(intersection.area()) / min_box_area : 0.0;
        if (overlap > 0.60) {
          if (area > existing.area) {
            existing = {name, box, area};
          }
          duplicate = true;
          break;
        }
      }
      if (!duplicate) {
        detections.push_back({name, box, area});
      }
    }
    return detections;
  }

  TemplateDetection matchTemplate(const cv::Mat & bgr) const
  {
    TemplateDetection detection;
    if (
      template_image_.empty() ||
      bgr.cols < template_image_.cols ||
      bgr.rows < template_image_.rows)
    {
      return detection;
    }

    cv::Scalar mean;
    cv::Scalar stddev;
    cv::meanStdDev(template_image_, mean, stddev);
    cv::Mat scores;
    if (stddev[0] < 1e-6 && stddev[1] < 1e-6 && stddev[2] < 1e-6) {
      cv::matchTemplate(bgr, template_image_, scores, cv::TM_SQDIFF_NORMED);
      double min_value = 0.0;
      cv::Point min_location;
      cv::minMaxLoc(scores, &min_value, nullptr, &min_location, nullptr);
      detection.score = 1.0 - min_value;
      detection.found = min_value <= (1.0 - match_threshold_);
      detection.location = min_location;
    } else {
      cv::matchTemplate(bgr, template_image_, scores, cv::TM_CCOEFF_NORMED);
      double max_value = 0.0;
      cv::Point max_location;
      cv::minMaxLoc(scores, nullptr, &max_value, nullptr, &max_location);
      detection.score = max_value;
      detection.found = max_value >= match_threshold_;
      detection.location = max_location;
    }
    detection.size = template_image_.size();
    return detection;
  }

  void processImage(const cv::Mat & source, const std_msgs::msg::Header & header)
  {
    if (source.empty()) {
      return;
    }

    cv::Mat bgr;
    if (source.channels() == 1) {
      cv::cvtColor(source, bgr, cv::COLOR_GRAY2BGR);
    } else if (source.channels() == 4) {
      cv::cvtColor(source, bgr, cv::COLOR_BGRA2BGR);
    } else {
      bgr = source;
    }

    cv::Mat mask;
    const auto colors = detectColors(bgr, mask);
    const auto shapes = detectShapes(bgr);
    const auto templ = matchTemplate(bgr);
    cv::Mat annotated = bgr.clone();

    for (const auto & detection : colors) {
      cv::rectangle(annotated, detection.box, cv::Scalar(255, 255, 255), 2);
      std::ostringstream label;
      label << detection.name << " " << std::fixed << std::setprecision(0)
            << detection.area;
      cv::putText(
        annotated, label.str(), detection.box.tl() + cv::Point(0, -6),
        cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
    }

    int triangle_count = 0;
    int rectangle_count = 0;
    int circle_count = 0;
    for (const auto & detection : shapes) {
      if (detection.name == "triangle") {
        ++triangle_count;
      } else if (detection.name == "rectangle") {
        ++rectangle_count;
      } else if (detection.name == "circle") {
        ++circle_count;
      }
      cv::rectangle(annotated, detection.box, cv::Scalar(0, 220, 255), 2);
      cv::putText(
        annotated, detection.name, detection.box.tl() + cv::Point(0, 18),
        cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 220, 255), 2, cv::LINE_AA);
    }

    if (templ.size.width > 0 && templ.size.height > 0) {
      const cv::Rect bounds(0, 0, bgr.cols, bgr.rows);
      const cv::Rect box(templ.location, templ.size);
      if (bounds.contains(box.tl()) && bounds.contains(box.br() - cv::Point(1, 1))) {
        cv::rectangle(
          annotated, box,
          templ.found ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 120, 255), 2);
      }
    }
    std::ostringstream template_label;
    template_label << "template " << (templ.found ? "found " : "not_found ")
                   << std::fixed << std::setprecision(2) << templ.score;
    cv::putText(
      annotated, template_label.str(), cv::Point(20, 35),
      cv::FONT_HERSHEY_SIMPLEX, 0.7,
      templ.found ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 120, 255), 2, cv::LINE_AA);

    annotated_pub_->publish(*cv_bridge::CvImage(header, "bgr8", annotated).toImageMsg());
    mask_pub_->publish(*cv_bridge::CvImage(header, "mono8", mask).toImageMsg());

    std_msgs::msg::String summary;
    std::ostringstream json;
    json << "{\"colors\":{\"red\":" << countColor(colors, "red")
         << ",\"green\":" << countColor(colors, "green")
         << ",\"blue\":" << countColor(colors, "blue")
         << "},\"shapes\":{\"triangle\":" << triangle_count
         << ",\"rectangle\":" << rectangle_count
         << ",\"circle\":" << circle_count
         << "},\"template\":{\"found\":"
         << (templ.found ? "true" : "false")
         << ",\"score\":" << std::fixed << std::setprecision(3) << templ.score
         << ",\"x\":" << templ.location.x << ",\"y\":" << templ.location.y
         << "}}";
    summary.data = json.str();
    detections_pub_->publish(summary);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "detections: colors=%zu shapes=%zu template=%s score=%.3f",
      colors.size(), shapes.size(), templ.found ? "found" : "not_found", templ.score);
  }

  static int countColor(const std::vector<ColorDetection> & detections, const std::string & name)
  {
    return static_cast<int>(std::count_if(
      detections.begin(), detections.end(),
      [&name](const ColorDetection & detection) { return detection.name == name; }));
  }

  std::string input_topic_;
  std::string test_image_topic_;
  std::string annotated_topic_;
  std::string mask_topic_;
  std::string detections_topic_;
  std::string template_path_;
  bool use_test_image_{true};
  double publish_rate_{5.0};
  double match_threshold_{0.8};
  double min_contour_area_{150.0};
  cv::Mat template_image_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr test_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr annotated_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mask_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr detections_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr input_sub_;
  rclcpp::TimerBase::SharedPtr test_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OpenCvDemoNode>());
  rclcpp::shutdown();
  return 0;
}
