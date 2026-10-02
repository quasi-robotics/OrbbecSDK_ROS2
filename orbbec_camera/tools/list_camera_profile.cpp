#include <rclcpp/rclcpp.hpp>
#include <orbbec_camera/ob_camera_node_driver.h>
#include <orbbec_camera/ob_camera_node.h>
#include <orbbec_camera/utils.h>
#include <magic_enum/magic_enum.hpp>
#include <chrono>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

using namespace orbbec_camera;

namespace {

constexpr int kFirmwareLogDrainDelaySec = 5;
constexpr char kDocumentationUrl[] =
    "https://orbbec.github.io/OrbbecSDK_ROS2/en/source/camera_devices/6_benchmark/"
    "device_query_tools.html";

struct CliArgs {
  bool help = false;
  std::string serial_number;
  std::string device_preset;
  std::string sdk_log_level = "off";
};

void printUsage() {
  std::cout << "Usage:\n"
            << "ros2 run orbbec_camera list_camera_profile_mode_node --\\\n"
            << "      [--serial_number SN] [--device_preset PRESET]\n\n"
            << "Parameters:\n"
            << "  --serial_number SN  Select a specific camera by serial number.\n"
            << "  --device_preset PRESET\n"
            << "                      Load a device preset before listing profiles.\n"
            << "  --sdk_log_level LEVEL\n"
            << "                      SDK file log level: debug/info/warn/error/fatal/off "
               "(default: off).\n"
            << "  -h, --help          Show this help message.\n"
            << "Examples:\n"
            << "  ros2 run orbbec_camera list_camera_profile_mode_node -- --sdk_log_level debug\n\n"
            << "Documentation:\n"
            << "  " << kDocumentationUrl << "\n";
}

bool parseArgs(int argc, char** argv, CliArgs& args, std::string& error) {
  for (int i = 1; i < argc; ++i) {
    const std::string current = argv[i];
    if (current == "-h" || current == "--help") {
      args.help = true;
      return true;
    }

    if (current.rfind("--serial_number=", 0) == 0) {
      args.serial_number = current.substr(std::strlen("--serial_number="));
      if (args.serial_number.empty()) {
        error = "--serial_number requires a value";
        return false;
      }
      continue;
    }

    if (current == "--serial_number") {
      if (++i >= argc) {
        error = "--serial_number requires a value";
        return false;
      }
      args.serial_number = argv[i];
      continue;
    }

    if (current.rfind("--device_preset=", 0) == 0) {
      args.device_preset = current.substr(std::strlen("--device_preset="));
      if (args.device_preset.empty()) {
        error = "--device_preset requires a value";
        return false;
      }
      continue;
    }

    if (current == "--device_preset") {
      if (++i >= argc) {
        error = "--device_preset requires a value";
        return false;
      }
      args.device_preset = argv[i];
      if (args.device_preset.empty()) {
        error = "--device_preset requires a value";
        return false;
      }
      continue;
    }

    if (current.rfind("--sdk_log_level=", 0) == 0) {
      args.sdk_log_level = current.substr(std::strlen("--sdk_log_level="));
      continue;
    }
    if (current == "--sdk_log_level") {
      if (++i >= argc) {
        error = "--sdk_log_level requires a value";
        return false;
      }
      args.sdk_log_level = argv[i];
      continue;
    }

    error = "Unknown argument: " + current;
    return false;
  }

  const auto log_severity = obLogSeverityFromString(args.sdk_log_level);
  if (log_severity == OBLogSeverity::OB_LOG_SEVERITY_OFF && args.sdk_log_level != "off" &&
      args.sdk_log_level != "none") {
    error = "--sdk_log_level expects one of: debug, info, warn, error, fatal, off";
    return false;
  }

  return true;
}

void waitForFirmwareLogDrain() {
  std::cout << "Waiting " << kFirmwareLogDrainDelaySec << " seconds to keep firmware log alive..."
            << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(kFirmwareLogDrainDelaySec));
}

bool enableFirmwareLog(const std::shared_ptr<ob::Device>& device) {
  try {
    device->enableFirmwareLog(true);
    std::cout << "Firmware log enabled." << std::endl;
    return true;
  } catch (const ob::Error& e) {
    std::cerr << "Failed to enable firmware log: " << formatObErrorWithStatus(e) << std::endl;
  } catch (const std::exception& e) {
    std::cerr << "Failed to enable firmware log: " << e.what() << std::endl;
  }
  return false;
}

bool isSdkLogEnabled(const std::string& log_level) {
  return obLogSeverityFromString(log_level) != OBLogSeverity::OB_LOG_SEVERITY_OFF;
}

std::shared_ptr<ob::Device> initializeDevice(const std::string& serial_number) {
  auto context = std::make_shared<ob::Context>();
  auto device_list = context->queryDeviceList();
  if (!device_list || device_list->getCount() == 0) {
    std::cout << "No device found\nFor usage and troubleshooting, see: " << kDocumentationUrl
              << std::endl;
    return nullptr;
  }

  if (!serial_number.empty()) {
    return device_list->getDeviceBySN(serial_number.c_str(), OB_DEVICE_DEFAULT_ACCESS);
  }

  return device_list->getDevice(0, OB_DEVICE_DEFAULT_ACCESS);
}

}  // namespace

void listSensorProfiles(const std::shared_ptr<ob::Device>& device) {
  auto sensor_list = device->getSensorList();
  auto pid = device->getDeviceInfo()->getPid();
  for (size_t i = 0; i < sensor_list->getCount(); i++) {
    auto sensor = sensor_list->getSensor(i);
    auto profile_list = sensor->getStreamProfileList();
    for (size_t j = 0; j < profile_list->getCount(); j++) {
      auto origin_profile = profile_list->getProfile(j);
      if ((sensor->getType() == OB_SENSOR_DEPTH || sensor->getType() == OB_SENSOR_IR_LEFT ||
           sensor->getType() == OB_SENSOR_IR_RIGHT) &&
          isGemini301SeriesPID(pid)) {
        // Gemini 301 series
        auto profile = origin_profile->as<ob::VideoStreamProfile>();
        std::cout << magic_enum::enum_name(sensor->getType()) << " profile: " << profile->getWidth()
                  << "x" << profile->getHeight() << " " << profile->getFps() << "fps "
                  << magic_enum::enum_name(profile->getFormat())
                  << " | width: " << profile->getDecimationConfig().originWidth
                  << " height: " << profile->getDecimationConfig().originHeight
                  << " downscale:" << profile->getDecimationConfig().factor << std::endl;
      } else if (sensor->getType() == OB_SENSOR_COLOR ||
                 sensor->getType() == OB_SENSOR_COLOR_LEFT ||
                 sensor->getType() == OB_SENSOR_COLOR_RIGHT ||
                 sensor->getType() == OB_SENSOR_DEPTH || sensor->getType() == OB_SENSOR_IR ||
                 sensor->getType() == OB_SENSOR_IR_LEFT ||
                 sensor->getType() == OB_SENSOR_IR_RIGHT) {
        auto profile = origin_profile->as<ob::VideoStreamProfile>();
        std::cout << magic_enum::enum_name(sensor->getType()) << " profile: " << profile->getWidth()
                  << "x" << profile->getHeight() << " " << profile->getFps() << "fps "
                  << magic_enum::enum_name(profile->getFormat()) << std::endl;
      } else if (sensor->getType() == OB_SENSOR_ACCEL) {
        auto profile = origin_profile->as<ob::AccelStreamProfile>();
        std::cout << magic_enum::enum_name(sensor->getType())
                  << " profile: " << profile->getSampleRate() << "  full scale_range "
                  << profile->getFullScaleRange() << std::endl;
      } else if (sensor->getType() == OB_SENSOR_GYRO) {
        auto profile = origin_profile->as<ob::GyroStreamProfile>();
        std::cout << magic_enum::enum_name(sensor->getType())
                  << " profile: " << profile->getSampleRate() << "  full scale_range "
                  << profile->getFullScaleRange() << std::endl;
      } else if (sensor->getType() == OB_SENSOR_LIDAR) {
        auto profile = origin_profile->as<ob::LiDARStreamProfile>();
        std::cout << magic_enum::enum_name(sensor->getType())
                  << " scan rate: " << magic_enum::enum_name(profile->getScanRate())
                  << "  format:" << magic_enum::enum_name(profile->getFormat()) << std::endl;
      } else {
        std::cout << "Unknown profile: " << magic_enum::enum_name(sensor->getType()) << std::endl;
      }
    }
  }
}

void printDeviceProperties(const std::shared_ptr<ob::Device>& device) {
  if (!device->isPropertySupported(OB_STRUCT_CURRENT_DEPTH_ALG_MODE, OB_PERMISSION_READ_WRITE)) {
    std::cout << "Current device not support depth work mode!" << std::endl;
    return;
  }
  auto current_depth_mode = device->getCurrentDepthWorkMode();
  std::cout << "Current depth mode: " << current_depth_mode.name << std::endl;
  auto depth_mode_list = device->getDepthWorkModeList();
  std::cout << "Depth mode list: " << std::endl;
  for (uint32_t i = 0; i < depth_mode_list->getCount(); i++) {
    std::cout << "Depth_mode_list[" << i << "]: " << (*depth_mode_list)[i].name << std::endl;
  }
}

void printPreset(const std::shared_ptr<ob::Device>& device) {
  auto preset_list = device->getAvailablePresetList();
  if (!preset_list || preset_list->getCount() == 0) {
    return;
  }
  std::cout << "Preset list:" << std::endl;
  for (uint32_t i = 0; i < preset_list->getCount(); i++) {
    auto name = preset_list->getName(i);
    std::string version;
    try {
      const char* version_value = preset_list->getDepthWorkModeVersion(i);
      version = version_value == nullptr ? "" : version_value;
    } catch (...) {
      // Older firmware can enumerate presets without exposing version information.
    }
    std::cout << "Preset list[" << i << "]: " << name
              << (version.empty() ? "" : " (" + version + ")") << std::endl;
  }
}

bool loadDevicePreset(const std::shared_ptr<ob::Device>& device, const std::string& device_preset) {
  if (device_preset.empty()) {
    return true;
  }

  try {
    device->loadPreset(device_preset.c_str());
    std::cout << "Loaded device preset: " << device_preset << std::endl;
    return true;
  } catch (const ob::Error& e) {
    std::cerr << "Failed to load device preset: " << formatObErrorWithStatus(e) << std::endl;
  } catch (const std::exception& e) {
    std::cerr << "Failed to load device preset: " << e.what() << std::endl;
  }
  return false;
}

int main(int argc, char** argv) {
  CliArgs args;
  std::string parse_error;
  if (!parseArgs(argc, argv, args, parse_error)) {
    std::cerr << parse_error << std::endl;
    printUsage();
    return 1;
  }
  if (args.help) {
    printUsage();
    return 0;
  }

  const auto sdk_log_path =
      configureObSdkLoggerForTool("list_camera_profile_mode_node", args.sdk_log_level);
  if (!sdk_log_path.empty()) {
    std::cout << "SDK file log enabled: " << sdk_log_path << std::endl;
  }

  auto device = initializeDevice(args.serial_number);
  if (!device) {
    return -1;
  }
  bool firmware_log_enabled = false;
  if (isSdkLogEnabled(args.sdk_log_level)) {
    firmware_log_enabled = enableFirmwareLog(device);
  }
  if (!loadDevicePreset(device, args.device_preset)) {
    return -1;
  }
  listSensorProfiles(device);
  printDeviceProperties(device);
  printPreset(device);
  if (firmware_log_enabled) {
    waitForFirmwareLogDrain();
  }
  return 0;
}
