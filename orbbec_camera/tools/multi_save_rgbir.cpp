#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/header.hpp>

#include "orbbec_camera/ob_camera_node.h"
#include "orbbec_camera/utils.h"
#include "orbbec_camera_msgs/msg/metadata.hpp"

namespace orbbec_camera {
namespace tools {
namespace {

const std::array<std::string, 6> kSupportedStreamNames = {
    "color", "left_color", "right_color", "ir", "left_ir", "right_ir",
};
constexpr auto kStreamDiscoveryPollInterval = std::chrono::milliseconds(100);
constexpr auto kStreamDiscoveryStablePeriod = std::chrono::seconds(1);
constexpr auto kStreamDiscoveryTimeout = std::chrono::seconds(5);
constexpr size_t kMinimumPendingFrameLimit = 30;

struct StreamTopicInfo {
  std::string name;
  bool metadata_available = false;

  bool operator==(const StreamTopicInfo &other) const {
    return name == other.name && metadata_available == other.metadata_available;
  }
};

bool isSupportedStreamName(const std::string &stream_name) {
  return std::find(kSupportedStreamNames.begin(), kSupportedStreamNames.end(), stream_name) !=
         kSupportedStreamNames.end();
}

bool isColorCaptureStreamName(const std::string &stream_name) {
  return stream_name == "color" || stream_name == "left_color" || stream_name == "right_color";
}

std::string cameraNamespace(const std::string &camera_name) {
  if (!camera_name.empty() && camera_name.front() == '/') {
    return camera_name;
  }
  return "/" + camera_name;
}

}  // namespace

struct StreamCapture {
  struct FrameMetadata {
    std::string exposure;
    std::string gain;
  };

  struct PendingImage {
    cv::Mat image;
    std::string current_timestamp;
    std::string receive_timestamp;
  };

  std::vector<cv::Mat> images;
  std::vector<std::string> current_timestamps;
  std::vector<std::string> receive_timestamps;
  std::vector<FrameMetadata> frame_metadata;
  std::map<int64_t, PendingImage> pending_images;
  std::map<int64_t, FrameMetadata> pending_metadata;
  bool metadata_required = false;

  void clear() {
    images.clear();
    current_timestamps.clear();
    receive_timestamps.clear();
    frame_metadata.clear();
    pending_images.clear();
    pending_metadata.clear();
  }
};

class MultiCameraSubscriber : public rclcpp::Node {
 public:
  explicit MultiCameraSubscriber(const rclcpp::NodeOptions &options)
      : Node("MultiCameraSubscriber", options) {
    initializeDeviceInfo();
    loadParameters();
    for (size_t i = 0; i < usb_ports_.size(); ++i) {
      usb_index_map_[usb_ports_[i]] = static_cast<int>(i);
    }
    for (const auto &entry : serial_numbers_) {
      RCLCPP_INFO(get_logger(), "usb_port: %s, serial: %s", entry.first.c_str(),
                  entry.second.c_str());
    }
    capture_control_srv_ = this->create_service<orbbec_camera_msgs::srv::SetInt32>(
        "start_capture", std::bind(&MultiCameraSubscriber::controlCaptureCallback, this,
                                   std::placeholders::_1, std::placeholders::_2));
  }

 private:
  void initializeDeviceInfo() {
    try {
      auto context = std::make_unique<ob::Context>();
      context->setLoggerSeverity(OBLogSeverity::OB_LOG_SEVERITY_NONE);
      auto list = context->queryDeviceList();
      for (size_t i = 0; i < list->deviceCount(); ++i) {
        auto device_info = list->getDevice(i)->getDeviceInfo();
        const auto usb_port = parseUsbPort(device_info->uid());
        serial_numbers_[usb_port] = device_info->serialNumber();
      }
    } catch (const ob::Error &e) {
      RCLCPP_ERROR_STREAM(get_logger(), orbbec_camera::formatObErrorWithStatus(e));
    } catch (const std::exception &e) {
      RCLCPP_ERROR_STREAM(get_logger(), e.what());
    } catch (...) {
      RCLCPP_ERROR(get_logger(), "unknown error while querying devices");
    }
  }

  void loadParameters() {
    std::ifstream file(
        "install/orbbec_camera/share/orbbec_camera/config/tools/multisavergbir/"
        "multi_save_rgbir_params.json");
    if (!file.is_open()) {
      RCLCPP_ERROR(get_logger(), "Failed to open JSON file.");
      return;
    }

    nlohmann::json json_data;
    file >> json_data;
    const auto &params = json_data["save_rgbir_params"];
    const auto time_domain = params["time_domain"].get<std::string>();
    time_domain_suffix_ =
        time_domain == "device" ? "_d" : (time_domain == "global" ? "_g" : "_unknown");
    usb_ports_ = params["usb_ports"].get<std::vector<std::string>>();
    camera_names_ = params["camera_name"].get<std::vector<std::string>>();

    if (params.contains("stream_names")) {
      for (const auto &stream_name : params["stream_names"].get<std::vector<std::string>>()) {
        if (!isSupportedStreamName(stream_name)) {
          throw std::invalid_argument("Unsupported stream name in multi_save_rgbir config: " +
                                      stream_name);
        }
        if (std::find(configured_stream_names_.begin(), configured_stream_names_.end(),
                      stream_name) == configured_stream_names_.end()) {
          configured_stream_names_.push_back(stream_name);
        }
      }
    }
  }

  std::vector<StreamTopicInfo> discoverStreams(const std::string &camera_name) const {
    std::vector<StreamTopicInfo> streams;
    const auto names_and_types = this->get_topic_names_and_types();
    const std::string prefix = cameraNamespace(camera_name) + "/";
    for (const auto &stream_name : kSupportedStreamNames) {
      const auto image_topic_it = names_and_types.find(prefix + stream_name + "/image_raw");
      if (image_topic_it == names_and_types.end()) {
        continue;
      }
      const auto &image_types = image_topic_it->second;
      if (std::find(image_types.begin(), image_types.end(), "sensor_msgs/msg/Image") ==
          image_types.end()) {
        continue;
      }

      const auto metadata_topic_it = names_and_types.find(prefix + stream_name + "/metadata");
      const bool metadata_available =
          metadata_topic_it != names_and_types.end() &&
          std::find(metadata_topic_it->second.begin(), metadata_topic_it->second.end(),
                    "orbbec_camera_msgs/msg/Metadata") != metadata_topic_it->second.end();
      streams.push_back(StreamTopicInfo{stream_name, metadata_available});
    }
    return streams;
  }

  std::vector<std::vector<StreamTopicInfo>> waitForStableStreams() const {
    if (!configured_stream_names_.empty()) {
      std::vector<std::vector<StreamTopicInfo>> configured_streams;
      configured_streams.reserve(camera_names_.size());
      for (const auto &camera_name : camera_names_) {
        const auto discovered_streams = discoverStreams(camera_name);
        std::vector<StreamTopicInfo> camera_streams;
        camera_streams.reserve(configured_stream_names_.size());
        for (const auto &stream_name : configured_stream_names_) {
          const auto discovered_it = std::find_if(
              discovered_streams.begin(), discovered_streams.end(),
              [&stream_name](const auto &stream) { return stream.name == stream_name; });
          camera_streams.push_back(StreamTopicInfo{
              stream_name,
              discovered_it != discovered_streams.end() && discovered_it->metadata_available});
        }
        configured_streams.push_back(std::move(camera_streams));
      }
      return configured_streams;
    }

    std::vector<std::vector<StreamTopicInfo>> candidate;
    auto candidate_since = std::chrono::steady_clock::time_point{};
    const auto deadline = std::chrono::steady_clock::now() + kStreamDiscoveryTimeout;
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      std::vector<std::vector<StreamTopicInfo>> current;
      current.reserve(camera_names_.size());
      bool all_cameras_discovered = !camera_names_.empty();
      for (const auto &camera_name : camera_names_) {
        current.push_back(discoverStreams(camera_name));
        all_cameras_discovered = all_cameras_discovered && !current.back().empty();
      }

      const auto now = std::chrono::steady_clock::now();
      if (!all_cameras_discovered) {
        candidate.clear();
      } else if (current != candidate) {
        candidate = std::move(current);
        candidate_since = now;
      } else if (now - candidate_since >= kStreamDiscoveryStablePeriod) {
        return candidate;
      }

      std::this_thread::sleep_for(kStreamDiscoveryPollInterval);
    }

    RCLCPP_WARN_STREAM(get_logger(),
                       "Supported image topics did not become stable within "
                           << kStreamDiscoveryTimeout.count()
                           << " seconds; start all cameras and streams first, retry the request, "
                              "or configure stream_names explicitly");
    return {};
  }

  bool initializeTopics() {
    const auto streams_by_camera = waitForStableStreams();
    if (streams_by_camera.size() != camera_names_.size()) {
      return false;
    }

    callback_groups_.clear();
    image_subscribers_.clear();
    metadata_subscribers_.clear();
    captures_.resize(camera_names_.size());
    callback_called_.assign(camera_names_.size(), false);
    const auto custom_qos =
        rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));

    for (size_t camera_index = 0; camera_index < camera_names_.size(); ++camera_index) {
      const auto &streams = streams_by_camera[camera_index];
      const std::string prefix = cameraNamespace(camera_names_[camera_index]) + "/";
      auto callback_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
      callback_groups_.push_back(callback_group);
      rclcpp::SubscriptionOptions options;
      options.callback_group = callback_group;

      for (const auto &stream : streams) {
        const auto &stream_name = stream.name;
        StreamCapture capture;
        capture.metadata_required = stream.metadata_available;
        captures_[camera_index].emplace(stream_name, std::move(capture));
        const std::string image_topic = prefix + stream_name + "/image_raw";
        RCLCPP_INFO(get_logger(), "Subscribing to %s", image_topic.c_str());

        image_subscribers_.push_back(this->create_subscription<sensor_msgs::msg::Image>(
            image_topic, custom_qos,
            [this, camera_index,
             stream_name](const std::shared_ptr<const sensor_msgs::msg::Image> image) {
              imageCallback(image, camera_index, stream_name);
            },
            options));
        if (stream.metadata_available) {
          const std::string metadata_topic = prefix + stream_name + "/metadata";
          metadata_subscribers_.push_back(
              this->create_subscription<orbbec_camera_msgs::msg::Metadata>(
                  metadata_topic, custom_qos,
                  [this, camera_index, stream_name](
                      const std::shared_ptr<const orbbec_camera_msgs::msg::Metadata> metadata) {
                    metadataCallback(metadata, camera_index, stream_name);
                  },
                  options));
        }
      }
    }
    return !captures_.empty() && std::all_of(captures_.begin(), captures_.end(),
                                             [](const auto &streams) { return !streams.empty(); });
  }

  std::string currentDateTime() const {
    const auto now = std::chrono::system_clock::now();
    const auto now_time = std::chrono::system_clock::to_time_t(now);
    const std::tm time_info = *std::localtime(&now_time);
    std::ostringstream output;
    output << std::put_time(&time_info, "%Y%m%d%H%M%S");
    return output.str();
  }

  std::string generateFolderName(const std::string &serial_number, size_t serial_index) const {
    const std::string path = "multicamera_sync/output/" + current_date_time_ +
                             "/TotalModeFrames/SN" + serial_number + "_Index" +
                             std::to_string(serial_index);
    std::filesystem::create_directories(path);
    return path;
  }

  std::string receiveTimestamp() {
    const auto now = this->get_clock()->now();
    const int64_t seconds = now.seconds();
    const int64_t milliseconds = now.nanoseconds() % 1000000000 / 1000000;
    return std::to_string(seconds) + std::to_string(milliseconds);
  }

  std::string imageTimestamp(const sensor_msgs::msg::Image::ConstSharedPtr &image) const {
    const int64_t milliseconds = image->header.stamp.nanosec / 1000000;
    std::ostringstream timestamp;
    timestamp << image->header.stamp.sec << std::setw(3) << std::setfill('0') << milliseconds;
    return timestamp.str();
  }

  bool captureReady(size_t camera_index) const {
    if (camera_index >= captures_.size() || captures_[camera_index].empty()) {
      return false;
    }
    return std::all_of(
        captures_[camera_index].begin(), captures_[camera_index].end(), [this](const auto &entry) {
          return entry.second.images.size() >= static_cast<size_t>(saving_images_number_);
        });
  }

  int64_t messageStampNs(const std_msgs::msg::Header &header) const {
    return static_cast<int64_t>(header.stamp.sec) * 1000000000LL + header.stamp.nanosec;
  }

  size_t pendingFrameLimit() const {
    return std::max(kMinimumPendingFrameLimit, static_cast<size_t>(saving_images_number_) * 2);
  }

  template <typename Value>
  void trimPendingFrames(std::map<int64_t, Value> &pending) const {
    while (pending.size() > pendingFrameLimit()) {
      pending.erase(pending.begin());
    }
  }

  void appendCompletedFrame(StreamCapture &capture, StreamCapture::PendingImage pending_image,
                            StreamCapture::FrameMetadata metadata = {}) {
    if (capture.images.size() >= static_cast<size_t>(saving_images_number_)) {
      return;
    }
    capture.images.push_back(std::move(pending_image.image));
    capture.current_timestamps.push_back(std::move(pending_image.current_timestamp));
    capture.receive_timestamps.push_back(std::move(pending_image.receive_timestamp));
    capture.frame_metadata.push_back(std::move(metadata));
  }

  std::string metadataSuffix(const StreamCapture &capture, size_t frame_index) const {
    if (frame_index >= capture.frame_metadata.size()) {
      return "";
    }
    std::string suffix;
    if (!capture.frame_metadata[frame_index].exposure.empty()) {
      suffix += "_e" + capture.frame_metadata[frame_index].exposure;
    }
    if (!capture.frame_metadata[frame_index].gain.empty()) {
      suffix += "_d" + capture.frame_metadata[frame_index].gain;
    }
    return suffix;
  }

  void saveImages(size_t camera_index) {
    if (!captureReady(camera_index)) {
      return;
    }
    if (camera_index >= usb_ports_.size()) {
      RCLCPP_ERROR(get_logger(), "Missing USB port configuration for camera index %zu",
                   camera_index);
      return;
    }
    const auto serial_it = serial_numbers_.find(usb_ports_[camera_index]);
    if (serial_it == serial_numbers_.end()) {
      RCLCPP_ERROR(get_logger(), "No serial number found for USB port %s",
                   usb_ports_[camera_index].c_str());
      return;
    }
    const auto usb_index_it = usb_index_map_.find(usb_ports_[camera_index]);
    const size_t usb_index = usb_index_it == usb_index_map_.end()
                                 ? camera_index
                                 : static_cast<size_t>(usb_index_it->second);
    const std::string &serial_number = serial_it->second;
    const std::string folder = generateFolderName(serial_number, usb_index);
    callback_called_[camera_index] = true;

    for (const auto &entry : captures_[camera_index]) {
      const std::string &stream_name = entry.first;
      const auto &capture = entry.second;
      for (size_t i = 0; i < static_cast<size_t>(saving_images_number_); ++i) {
        if (capture.images[i].empty()) {
          continue;
        }
        const std::string filename = folder + "/" + stream_name + "_SN" + serial_number + "_Index" +
                                     std::to_string(usb_index) + time_domain_suffix_ +
                                     capture.current_timestamps[i] + "_f" + std::to_string(i) +
                                     "_s" + capture.receive_timestamps[i] +
                                     metadataSuffix(capture, i) + "_.jpg";
        cv::imwrite(filename, capture.images[i]);
      }
    }

    for (auto &entry : captures_[camera_index]) {
      entry.second.clear();
    }
    const bool all_cameras_complete = std::all_of(callback_called_.begin(), callback_called_.end(),
                                                  [](bool value) { return value; });
    if (all_cameras_complete) {
      RCLCPP_INFO(get_logger(), "Capture completed for all cameras");
      saving_images_number_ = 0;
      callback_called_.assign(camera_names_.size(), false);
    }
  }

  void controlCaptureCallback(
      const std::shared_ptr<orbbec_camera_msgs::srv::SetInt32::Request> request,
      std::shared_ptr<orbbec_camera_msgs::srv::SetInt32::Response> response) {
    std::lock_guard<std::mutex> lock(capture_mutex_);
    if (request->data <= 0) {
      response->success = false;
      response->message = "capture image count must be greater than zero";
      return;
    }
    if (!topics_initialized_) {
      if (!initializeTopics()) {
        callback_groups_.clear();
        image_subscribers_.clear();
        metadata_subscribers_.clear();
        captures_.clear();
        callback_called_.clear();
        response->success = false;
        response->message =
            "supported image streams did not become stable; retry after all cameras and streams "
            "start or configure stream_names";
        return;
      }
      topics_initialized_ = true;
    }

    for (auto &camera_captures : captures_) {
      for (auto &entry : camera_captures) {
        entry.second.clear();
      }
    }
    callback_called_.assign(camera_names_.size(), false);
    current_date_time_ = currentDateTime();
    saving_images_number_ = request->data;
    response->success = true;
    response->message = "capture started";
    RCLCPP_INFO(get_logger(), "Capturing %d image(s) from each configured stream",
                saving_images_number_);
  }

  void imageCallback(const std::shared_ptr<const sensor_msgs::msg::Image> image,
                     size_t camera_index, const std::string &stream_name) {
    std::lock_guard<std::mutex> lock(capture_mutex_);
    if (saving_images_number_ <= 0 || callback_called_[camera_index]) {
      return;
    }
    auto &capture = captures_[camera_index].at(stream_name);
    if (capture.images.size() >= static_cast<size_t>(saving_images_number_)) {
      saveImages(camera_index);
      return;
    }
    cv::Mat output = cv_bridge::toCvCopy(image, image->encoding)->image;
    if (isColorCaptureStreamName(stream_name) &&
        image->encoding == sensor_msgs::image_encodings::RGB8) {
      cv::Mat converted;
      cv::cvtColor(output, converted, cv::COLOR_RGB2BGR);
      output = converted;
    } else if (isColorCaptureStreamName(stream_name) &&
               image->encoding == sensor_msgs::image_encodings::RGBA8) {
      cv::Mat converted;
      cv::cvtColor(output, converted, cv::COLOR_RGBA2BGRA);
      output = converted;
    }
    StreamCapture::PendingImage pending_image{std::move(output), imageTimestamp(image),
                                              receiveTimestamp()};
    if (capture.metadata_required) {
      const auto stamp_ns = messageStampNs(image->header);
      auto metadata_it = capture.pending_metadata.find(stamp_ns);
      if (metadata_it == capture.pending_metadata.end()) {
        capture.pending_images.insert_or_assign(stamp_ns, std::move(pending_image));
        trimPendingFrames(capture.pending_images);
        return;
      }
      appendCompletedFrame(capture, std::move(pending_image), std::move(metadata_it->second));
      capture.pending_metadata.erase(metadata_it);
    } else {
      appendCompletedFrame(capture, std::move(pending_image));
    }
    RCLCPP_INFO(get_logger(), "%s[%zu]: %zu/%d", stream_name.c_str(), camera_index,
                capture.images.size(), saving_images_number_);
    saveImages(camera_index);
  }

  void metadataCallback(const std::shared_ptr<const orbbec_camera_msgs::msg::Metadata> metadata,
                        size_t camera_index, const std::string &stream_name) {
    std::lock_guard<std::mutex> lock(capture_mutex_);
    if (saving_images_number_ <= 0 || callback_called_[camera_index]) {
      return;
    }
    auto &capture = captures_[camera_index].at(stream_name);
    if (!capture.metadata_required ||
        capture.images.size() >= static_cast<size_t>(saving_images_number_)) {
      return;
    }
    StreamCapture::FrameMetadata frame_metadata;
    try {
      const auto json_data = nlohmann::json::parse(metadata->json_data);
      if (json_data.contains("exposure")) {
        frame_metadata.exposure = json_data["exposure"].dump();
      }
      if (json_data.contains("gain")) {
        frame_metadata.gain = json_data["gain"].dump();
      }
    } catch (const std::exception &e) {
      RCLCPP_WARN(get_logger(), "Failed to parse %s metadata: %s", stream_name.c_str(), e.what());
    }
    const auto stamp_ns = messageStampNs(metadata->header);
    auto image_it = capture.pending_images.find(stamp_ns);
    if (image_it == capture.pending_images.end()) {
      capture.pending_metadata.insert_or_assign(stamp_ns, std::move(frame_metadata));
      trimPendingFrames(capture.pending_metadata);
      return;
    }
    appendCompletedFrame(capture, std::move(image_it->second), std::move(frame_metadata));
    capture.pending_images.erase(image_it);
    RCLCPP_INFO(get_logger(), "%s[%zu]: %zu/%d", stream_name.c_str(), camera_index,
                capture.images.size(), saving_images_number_);
    saveImages(camera_index);
  }

  std::mutex capture_mutex_;
  std::vector<rclcpp::CallbackGroup::SharedPtr> callback_groups_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> image_subscribers_;
  std::vector<rclcpp::Subscription<orbbec_camera_msgs::msg::Metadata>::SharedPtr>
      metadata_subscribers_;
  rclcpp::Service<orbbec_camera_msgs::srv::SetInt32>::SharedPtr capture_control_srv_;

  std::map<std::string, int> usb_index_map_;
  std::map<std::string, std::string> serial_numbers_;
  std::vector<std::string> usb_ports_;
  std::vector<std::string> camera_names_;
  std::vector<std::string> configured_stream_names_;
  std::vector<std::map<std::string, StreamCapture>> captures_;
  std::vector<bool> callback_called_;
  std::string time_domain_suffix_;
  std::string current_date_time_;
  int saving_images_number_ = 0;
  bool topics_initialized_ = false;
};

}  // namespace tools
}  // namespace orbbec_camera

RCLCPP_COMPONENTS_REGISTER_NODE(orbbec_camera::tools::MultiCameraSubscriber)
