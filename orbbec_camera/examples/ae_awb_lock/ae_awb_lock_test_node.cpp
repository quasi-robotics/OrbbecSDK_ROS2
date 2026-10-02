// Copyright (c) 2026 Orbbec Inc. All Rights Reserved.
// Licensed under the Apache License, Version 2.0.

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
#include <thread>

#include "orbbec_camera_msgs/action/run_ae_awb_lock_test.hpp"
#include "orbbec_camera_msgs/msg/metadata.hpp"
#include "orbbec_camera_msgs/srv/get_awb_gain.hpp"
#include "orbbec_camera_msgs/srv/get_int32.hpp"
#include "orbbec_camera_msgs/srv/set_awb_gain.hpp"
#include "orbbec_camera_msgs/srv/set_int32.hpp"

namespace {

using namespace std::chrono_literals;

constexpr int32_t kAeAwbConverged = 1;
constexpr uint32_t kDefaultTimeoutMs = 10000;
constexpr auto kPollInterval = 100ms;
constexpr auto kRestoreServiceTimeout = 5s;

class CanceledError : public std::runtime_error {
 public:
  CanceledError() : std::runtime_error("goal canceled") {}
};

class AeAwbLockTestNode : public rclcpp::Node {
 public:
  using RunAeAwbLockTest = orbbec_camera_msgs::action::RunAeAwbLockTest;
  using GoalHandle = rclcpp_action::ServerGoalHandle<RunAeAwbLockTest>;
  using GetInt32 = orbbec_camera_msgs::srv::GetInt32;
  using SetInt32 = orbbec_camera_msgs::srv::SetInt32;
  using GetAwbGain = orbbec_camera_msgs::srv::GetAwbGain;
  using SetAwbGain = orbbec_camera_msgs::srv::SetAwbGain;
  using SetBool = std_srvs::srv::SetBool;
  using Metadata = orbbec_camera_msgs::msg::Metadata;
  using Deadline = std::chrono::steady_clock::time_point;

  AeAwbLockTestNode() : Node("ae_awb_lock_test_node") {
    color_metadata_subscription_ = create_subscription<Metadata>(
        "color/metadata", rclcpp::SensorDataQoS(),
        std::bind(&AeAwbLockTestNode::colorMetadataCallback, this, std::placeholders::_1));

    get_status_client_ = create_client<GetInt32>("get_color_ae_awb_status");
    get_awb_gain_client_ = create_client<GetAwbGain>("get_color_awb_gain");
    set_awb_gain_client_ = create_client<SetAwbGain>("set_color_awb_gain");
    get_exposure_client_ = create_client<GetInt32>("get_color_exposure");
    set_exposure_client_ = create_client<SetInt32>("set_color_exposure");
    get_color_gain_client_ = create_client<GetInt32>("get_color_gain");
    set_color_gain_client_ = create_client<SetInt32>("set_color_gain");
    get_white_balance_client_ = create_client<GetInt32>("get_white_balance");
    set_white_balance_client_ = create_client<SetInt32>("set_white_balance");
    set_auto_exposure_client_ = create_client<SetBool>("set_color_auto_exposure");
    set_auto_white_balance_client_ = create_client<SetBool>("set_auto_white_balance");

    action_server_ = rclcpp_action::create_server<RunAeAwbLockTest>(
        this, "run_ae_awb_lock_test",
        std::bind(&AeAwbLockTestNode::handleGoal, this, std::placeholders::_1,
                  std::placeholders::_2),
        std::bind(&AeAwbLockTestNode::handleCancel, this, std::placeholders::_1),
        std::bind(&AeAwbLockTestNode::handleAccepted, this, std::placeholders::_1));
  }

  ~AeAwbLockTestNode() override {
    shutting_down_.store(true);
    metadata_cv_.notify_all();
    std::lock_guard<std::mutex> lock(worker_mutex_);
    if (worker_.joinable()) {
      worker_.join();
    }
  }

 private:
  struct ColorFrameMetadata {
    int32_t exposure;
    int32_t gain;
    int32_t white_balance;
  };

  struct ActiveGoalGuard {
    explicit ActiveGoalGuard(std::atomic_bool& active) : active_(active) {}
    ~ActiveGoalGuard() { active_.store(false); }
    std::atomic_bool& active_;
  };

  rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID& uuid,
                                         std::shared_ptr<const RunAeAwbLockTest::Goal> goal) {
    (void)uuid;
    (void)goal;
    bool expected = false;
    if (!goal_active_.compare_exchange_strong(expected, true)) {
      RCLCPP_WARN(get_logger(), "Rejecting AE/AWB test goal: another goal is active");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle> goal_handle) {
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle) {
    std::lock_guard<std::mutex> lock(worker_mutex_);
    if (worker_.joinable()) {
      worker_.join();
    }
    worker_ = std::thread(&AeAwbLockTestNode::execute, this, goal_handle);
  }

  template <typename ServiceT>
  void waitForService(const typename rclcpp::Client<ServiceT>::SharedPtr& client,
                      const std::string& service_name, const Deadline& deadline) {
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline || !client->wait_for_service(deadline - now)) {
      throw std::runtime_error(service_name + " is not available");
    }
  }

  // Foxy has no public API for removing timed-out requests. Newer rclcpp versions do, so use it
  // when available without making the example depend on a particular ROS distribution.
  template <typename ClientT, typename PendingRequestT>
  static auto removePendingRequestIfSupported(const ClientT& client,
                                              const PendingRequestT& pending_request, int)
      -> decltype(client->remove_pending_request(pending_request), void()) {
    client->remove_pending_request(pending_request);
  }

  template <typename ClientT, typename PendingRequestT>
  static void removePendingRequestIfSupported(const ClientT&, const PendingRequestT&, long) {}

  template <typename ServiceT>
  typename ServiceT::Response::SharedPtr callService(
      const typename rclcpp::Client<ServiceT>::SharedPtr& client,
      const typename ServiceT::Request::SharedPtr& request, const std::string& service_name,
      const Deadline& deadline) {
    auto pending_request = client->async_send_request(request);
    if (pending_request.wait_until(deadline) != std::future_status::ready) {
      removePendingRequestIfSupported(client, pending_request, 0);
      throw std::runtime_error(service_name + " timed out");
    }
    auto response = pending_request.get();
    if (!response->success) {
      throw std::runtime_error(service_name + " failed: " + response->message);
    }
    return response;
  }

  void waitForRequiredServices(const Deadline& deadline) {
    waitForService<GetInt32>(get_status_client_, "get_color_ae_awb_status", deadline);
    waitForService<GetAwbGain>(get_awb_gain_client_, "get_color_awb_gain", deadline);
    waitForService<SetAwbGain>(set_awb_gain_client_, "set_color_awb_gain", deadline);
    waitForService<GetInt32>(get_exposure_client_, "get_color_exposure", deadline);
    waitForService<SetInt32>(set_exposure_client_, "set_color_exposure", deadline);
    waitForService<GetInt32>(get_color_gain_client_, "get_color_gain", deadline);
    waitForService<SetInt32>(set_color_gain_client_, "set_color_gain", deadline);
    waitForService<GetInt32>(get_white_balance_client_, "get_white_balance", deadline);
    waitForService<SetInt32>(set_white_balance_client_, "set_white_balance", deadline);
    waitForService<SetBool>(set_auto_exposure_client_, "set_color_auto_exposure", deadline);
    waitForService<SetBool>(set_auto_white_balance_client_, "set_auto_white_balance", deadline);
  }

  int32_t getIntValue(const rclcpp::Client<GetInt32>::SharedPtr& client,
                      const std::string& service_name, const Deadline& deadline) {
    return callService<GetInt32>(client, std::make_shared<GetInt32::Request>(), service_name,
                                 deadline)
        ->data;
  }

  int32_t getAeAwbStatus(const Deadline& deadline) {
    return getIntValue(get_status_client_, "get_color_ae_awb_status", deadline);
  }

  void colorMetadataCallback(Metadata::ConstSharedPtr metadata) {
    {
      std::lock_guard<std::mutex> lock(metadata_mutex_);
      latest_color_metadata_ = metadata;
    }
    metadata_cv_.notify_all();
  }

  ColorFrameMetadata getLatestColorMetadata(const Deadline& deadline) {
    Metadata::ConstSharedPtr metadata;
    {
      std::unique_lock<std::mutex> lock(metadata_mutex_);
      if (!metadata_cv_.wait_until(lock, deadline, [this] {
            return latest_color_metadata_ != nullptr || shutting_down_.load();
          })) {
        throw std::runtime_error("timed out waiting for color frame metadata");
      }
      if (shutting_down_.load()) {
        throw CanceledError();
      }
      metadata = latest_color_metadata_;
    }

    try {
      const auto json = nlohmann::json::parse(metadata->json_data);
      return {json.at("exposure").get<int32_t>(), json.at("gain").get<int32_t>(),
              json.at("white_balance").get<int32_t>()};
    } catch (const nlohmann::json::exception& e) {
      throw std::runtime_error(std::string("invalid color frame metadata: ") + e.what());
    }
  }

  void setIntValue(const rclcpp::Client<SetInt32>::SharedPtr& client,
                   const std::string& service_name, int32_t value, const Deadline& deadline) {
    auto request = std::make_shared<SetInt32::Request>();
    request->data = value;
    callService<SetInt32>(client, request, service_name, deadline);
  }

  GetAwbGain::Response::SharedPtr getAwbGain(const Deadline& deadline) {
    return callService<GetAwbGain>(get_awb_gain_client_, std::make_shared<GetAwbGain::Request>(),
                                   "get_color_awb_gain", deadline);
  }

  void setAwbGain(uint16_t r_gain, uint16_t b_gain, uint16_t g_gain, const Deadline& deadline) {
    auto request = std::make_shared<SetAwbGain::Request>();
    request->r_gain = r_gain;
    request->b_gain = b_gain;
    request->g_gain = g_gain;
    callService<SetAwbGain>(set_awb_gain_client_, request, "set_color_awb_gain", deadline);
  }

  void setBoolValue(const rclcpp::Client<SetBool>::SharedPtr& client,
                    const std::string& service_name, bool value, const Deadline& deadline) {
    auto request = std::make_shared<SetBool::Request>();
    request->data = value;
    callService<SetBool>(client, request, service_name, deadline);
  }

  void setAutoMode(bool enabled, const Deadline& deadline) {
    setBoolValue(set_auto_exposure_client_, "set_color_auto_exposure", enabled, deadline);
    setBoolValue(set_auto_white_balance_client_, "set_auto_white_balance", enabled, deadline);
  }

  void restoreAutoMode() noexcept {
    try {
      const auto deadline = std::chrono::steady_clock::now() + kRestoreServiceTimeout;
      setBoolValue(set_auto_exposure_client_, "set_color_auto_exposure", true, deadline);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Failed to restore auto exposure: %s", e.what());
    }
    try {
      const auto deadline = std::chrono::steady_clock::now() + kRestoreServiceTimeout;
      setBoolValue(set_auto_white_balance_client_, "set_auto_white_balance", true, deadline);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Failed to restore auto white balance: %s", e.what());
    }
  }

  void throwIfCanceled(const std::shared_ptr<GoalHandle>& goal_handle) const {
    if (shutting_down_.load() || goal_handle->is_canceling()) {
      throw CanceledError();
    }
  }

  void publishFeedback(const std::shared_ptr<GoalHandle>& goal_handle, const std::string& phase,
                       int32_t status) {
    auto feedback = std::make_shared<RunAeAwbLockTest::Feedback>();
    feedback->phase = phase;
    feedback->ae_awb_status = status;
    goal_handle->publish_feedback(feedback);
  }

  void warnIfChanged(const char* name, int64_t captured, int64_t actual) {
    if (captured != actual) {
      RCLCPP_WARN(get_logger(), "%s changed from %lld to %lld after writeback", name,
                  static_cast<long long>(captured), static_cast<long long>(actual));
    }
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle) {
    ActiveGoalGuard active_guard(goal_active_);
    auto result = std::make_shared<RunAeAwbLockTest::Result>();
    bool workflow_started = false;
    const uint32_t requested_timeout = goal_handle->get_goal()->timeout_ms;
    const uint32_t timeout_ms = requested_timeout == 0 ? kDefaultTimeoutMs : requested_timeout;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);

    try {
      waitForRequiredServices(deadline);
      publishFeedback(goal_handle, "waiting_for_services", getAeAwbStatus(deadline));
      throwIfCanceled(goal_handle);

      workflow_started = true;
      setAutoMode(true, deadline);
      publishFeedback(goal_handle, "enabling_auto", getAeAwbStatus(deadline));

      int32_t status = 0;
      do {
        throwIfCanceled(goal_handle);
        if (std::chrono::steady_clock::now() >= deadline) {
          throw std::runtime_error("timed out waiting for AE/AWB convergence");
        }
        status = getAeAwbStatus(deadline);
        publishFeedback(goal_handle, "waiting_for_convergence", status);
        if (status != kAeAwbConverged) {
          std::this_thread::sleep_for(kPollInterval);
        }
      } while (status != kAeAwbConverged);

      throwIfCanceled(goal_handle);
      publishFeedback(goal_handle, "capturing_parameters", getAeAwbStatus(deadline));
      const auto captured_metadata = getLatestColorMetadata(deadline);
      result->captured_exposure = captured_metadata.exposure;
      result->captured_color_gain = captured_metadata.gain;
      result->captured_color_temperature = captured_metadata.white_balance;
      auto captured_awb_gain = getAwbGain(deadline);
      result->captured_awb_r_gain = captured_awb_gain->r_gain;
      result->captured_awb_b_gain = captured_awb_gain->b_gain;
      result->captured_awb_g_gain = captured_awb_gain->g_gain;

      throwIfCanceled(goal_handle);
      publishFeedback(goal_handle, "disabling_auto", getAeAwbStatus(deadline));
      setAutoMode(false, deadline);
      throwIfCanceled(goal_handle);

      publishFeedback(goal_handle, "applying_exposure", getAeAwbStatus(deadline));
      setIntValue(set_exposure_client_, "set_color_exposure", result->captured_exposure, deadline);
      throwIfCanceled(goal_handle);

      publishFeedback(goal_handle, "applying_color_gain", getAeAwbStatus(deadline));
      setIntValue(set_color_gain_client_, "set_color_gain", result->captured_color_gain, deadline);
      throwIfCanceled(goal_handle);

      publishFeedback(goal_handle, "applying_awb_gain", getAeAwbStatus(deadline));
      setAwbGain(result->captured_awb_r_gain, result->captured_awb_b_gain,
                 result->captured_awb_g_gain, deadline);
      throwIfCanceled(goal_handle);

      publishFeedback(goal_handle, "applying_color_temperature", getAeAwbStatus(deadline));
      setIntValue(set_white_balance_client_, "set_white_balance",
                  result->captured_color_temperature, deadline);
      throwIfCanceled(goal_handle);

      publishFeedback(goal_handle, "verifying", getAeAwbStatus(deadline));
      result->actual_exposure = getIntValue(get_exposure_client_, "get_color_exposure", deadline);
      result->actual_color_gain = getIntValue(get_color_gain_client_, "get_color_gain", deadline);
      auto actual_awb_gain = getAwbGain(deadline);
      result->actual_awb_r_gain = actual_awb_gain->r_gain;
      result->actual_awb_b_gain = actual_awb_gain->b_gain;
      result->actual_awb_g_gain = actual_awb_gain->g_gain;
      result->actual_color_temperature =
          getIntValue(get_white_balance_client_, "get_white_balance", deadline);
      throwIfCanceled(goal_handle);

      if (result->captured_awb_r_gain != result->actual_awb_r_gain ||
          result->captured_awb_b_gain != result->actual_awb_b_gain ||
          result->captured_awb_g_gain != result->actual_awb_g_gain) {
        throw std::runtime_error("AWB gain readback does not match the captured value");
      }

      warnIfChanged("Exposure", result->captured_exposure, result->actual_exposure);
      warnIfChanged("Color gain", result->captured_color_gain, result->actual_color_gain);
      warnIfChanged("Color temperature", result->captured_color_temperature,
                    result->actual_color_temperature);

      result->success = true;
      result->message = "AE/AWB capture and lock-in completed";
      publishFeedback(goal_handle, "completed", getAeAwbStatus(deadline));
      goal_handle->succeed(result);
    } catch (const CanceledError&) {
      if (workflow_started) {
        restoreAutoMode();
      }
      result->success = false;
      result->message = "AE/AWB test canceled";
      goal_handle->canceled(result);
    } catch (const std::exception& e) {
      if (workflow_started) {
        restoreAutoMode();
      }
      result->success = false;
      result->message = e.what();
      RCLCPP_ERROR(get_logger(), "AE/AWB test failed: %s", e.what());
      goal_handle->abort(result);
    }
  }

  rclcpp_action::Server<RunAeAwbLockTest>::SharedPtr action_server_;
  rclcpp::Subscription<Metadata>::SharedPtr color_metadata_subscription_;

  rclcpp::Client<GetInt32>::SharedPtr get_status_client_;
  rclcpp::Client<GetAwbGain>::SharedPtr get_awb_gain_client_;
  rclcpp::Client<SetAwbGain>::SharedPtr set_awb_gain_client_;
  rclcpp::Client<GetInt32>::SharedPtr get_exposure_client_;
  rclcpp::Client<SetInt32>::SharedPtr set_exposure_client_;
  rclcpp::Client<GetInt32>::SharedPtr get_color_gain_client_;
  rclcpp::Client<SetInt32>::SharedPtr set_color_gain_client_;
  rclcpp::Client<GetInt32>::SharedPtr get_white_balance_client_;
  rclcpp::Client<SetInt32>::SharedPtr set_white_balance_client_;
  rclcpp::Client<SetBool>::SharedPtr set_auto_exposure_client_;
  rclcpp::Client<SetBool>::SharedPtr set_auto_white_balance_client_;

  std::atomic_bool goal_active_{false};
  std::atomic_bool shutting_down_{false};
  Metadata::ConstSharedPtr latest_color_metadata_;
  std::mutex metadata_mutex_;
  std::condition_variable metadata_cv_;
  std::mutex worker_mutex_;
  std::thread worker_;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AeAwbLockTestNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
