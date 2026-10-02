#include <gtest/gtest.h>

#include <cstring>
#include <vector>

#include "orbbec_camera/utils.h"

namespace orbbec_camera {
namespace {

OBCameraIntrinsic makeIntrinsic() {
  OBCameraIntrinsic intrinsic{};
  intrinsic.width = 1280;
  intrinsic.height = 800;
  intrinsic.fx = 600.0F;
  intrinsic.fy = 601.0F;
  intrinsic.cx = 640.0F;
  intrinsic.cy = 400.0F;
  return intrinsic;
}

OBCameraDistortion makeDistortion(OBCameraDistortionModel model) {
  OBCameraDistortion distortion{};
  distortion.k1 = 0.1F;
  distortion.k2 = 0.2F;
  distortion.k3 = 0.3F;
  distortion.k4 = 0.4F;
  distortion.k5 = 0.5F;
  distortion.k6 = 0.6F;
  distortion.p1 = 0.01F;
  distortion.p2 = 0.02F;
  distortion.model = model;
  return distortion;
}

std::shared_ptr<ob::ColorFrame> makeMjpegFrame(const std::vector<uint8_t>& data) {
  auto frame = ob::FrameFactory::createFrame(OB_FRAME_COLOR, OB_FORMAT_MJPG,
                                             static_cast<uint32_t>(data.size()));
  auto color_frame = frame->as<ob::ColorFrame>();
  std::memcpy(color_frame->getData(), data.data(), data.size());
  return color_frame;
}

TEST(CameraInfoDistortionTest, ConvertsBrownConradyToPlumbBob) {
  const auto intrinsic = makeIntrinsic();
  const auto distortion = makeDistortion(OB_DISTORTION_BROWN_CONRADY);

  const auto info = convertToCameraInfo(intrinsic, distortion, intrinsic.width);

  EXPECT_EQ(info.distortion_model, sensor_msgs::distortion_models::PLUMB_BOB);
  EXPECT_EQ(info.d, std::vector<double>({distortion.k1, distortion.k2, distortion.p1, distortion.p2,
                                         distortion.k3}));
}

TEST(CameraInfoDistortionTest, KeepsK6ModelWhenHigherOrderCoefficientsAreZero) {
  const auto intrinsic = makeIntrinsic();
  auto distortion = makeDistortion(OB_DISTORTION_BROWN_CONRADY_K6);
  distortion.k4 = 0.0F;
  distortion.k5 = 0.0F;
  distortion.k6 = 0.0F;

  const auto info = convertToCameraInfo(intrinsic, distortion, intrinsic.width);

  EXPECT_EQ(info.distortion_model, sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL);
  EXPECT_EQ(info.d,
            std::vector<double>({distortion.k1, distortion.k2, distortion.p1, distortion.p2,
                                 distortion.k3, distortion.k4, distortion.k5, distortion.k6}));
}

TEST(CameraInfoDistortionTest, ConvertsKannalaBrandtToEquidistant) {
  const auto intrinsic = makeIntrinsic();
  const auto distortion = makeDistortion(OB_DISTORTION_KANNALA_BRANDT4);

  const auto info = convertToCameraInfo(intrinsic, distortion, intrinsic.width);

  EXPECT_EQ(info.distortion_model, sensor_msgs::distortion_models::EQUIDISTANT);
  EXPECT_EQ(info.d,
            std::vector<double>({distortion.k1, distortion.k2, distortion.k3, distortion.k4}));
}

TEST(JpegValidationTest, AcceptsEoiBeforeZeroPadding) {
  const auto frame = makeMjpegFrame({0xFF, 0xD8, 0x01, 0x02, 0xFF, 0xD9, 0x00, 0x00});

  EXPECT_TRUE(isValidJPEG(frame));
}

TEST(JpegValidationTest, RejectsMjpegWithoutEoi) {
  const auto frame = makeMjpegFrame({0xFF, 0xD8, 0x01, 0x02, 0x00, 0x00});

  EXPECT_FALSE(isValidJPEG(frame));
}

}  // namespace
}  // namespace orbbec_camera
