// Invoke the actual linked SlamToolbox method. No scan/TF/command publishers.
#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include "slam_toolbox/slam_toolbox_common.hpp"

class GateProbe : public slam_toolbox::SlamToolbox {
public:
  GateProbe() : SlamToolbox(rclcpp::NodeOptions().enable_rosout(false)
      .start_parameter_services(false).start_parameter_event_publisher(false)) {
    smapper_->getMapper()->setParamMinimumTravelDistance(.01);
    smapper_->getMapper()->setParamMinimumTravelHeading(.02);
    minimum_time_interval_ = rclcpp::Duration::from_seconds(.05);
    throttle_scans_ = 1;
    state_.set(toolbox_types::NEW_MEASUREMENTS, false);
    std::cout << "PARAMS distance=" << smapper_->getMapper()->getParamMinimumTravelDistance()
              << " heading_degrees=" << smapper_->getMapper()->getParamMinimumTravelHeading() << "\n";
  }
  bool sample(double seconds, double x, double y, double heading) {
    auto scan = std::make_shared<sensor_msgs::msg::LaserScan>();
    scan->header.stamp = rclcpp::Time(static_cast<int64_t>(seconds * 1.e9));
    return shouldProcessScan(scan, karto::Pose2(x, y, heading));
  }
  void pause(bool value) { state_.set(toolbox_types::NEW_MEASUREMENTS, value); }
  void throttle(int value) { throttle_scans_ = value; }
  void laserCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr) override {}
};

int main(int argc, char **argv) {
  if (argc != 3) return 2;
  const bool patched = std::string(argv[1]) == "patched";
  const std::string scenario(argv[2]);
  rclcpp::init(0, nullptr);
  int checked = 0;
  auto expect = [&](bool actual, bool wanted) {
    ++checked;
    if (actual != wanted) throw std::runtime_error("decision " + std::to_string(checked));
  };
  try {
    auto probe = std::make_shared<GateProbe>();
    const double initial = scenario == "wrap" ? 3.135 : 0.;
    expect(probe->sample(10., 0., 0., initial), true);
    for (int i = 1; i <= 3; ++i)
      expect(probe->sample(10. + i * .1, 0., 0., initial), false);
    if (scenario == "stationary") {
      for (int i = 0; i < 600; ++i)
        expect(probe->sample(11. + i * .1, 0., 0., 0.), false);
    } else if (scenario == "positive" || scenario == "negative") {
      const double heading = scenario == "positive" ? .03 : -.03;
      expect(probe->sample(11., 0., 0., heading), patched);
      expect(probe->sample(12., 0., 0., heading), false);
    } else if (scenario == "accumulation") {
      expect(probe->sample(11., 0., 0., .015), false);
      expect(probe->sample(12., 0., 0., .021), patched);
      expect(probe->sample(13., 0., 0., .029), false);
      expect(probe->sample(14., 0., 0., .042), patched);
    } else if (scenario == "wrap") {
      expect(probe->sample(11., 0., 0., -3.135), false);
      expect(probe->sample(12., 0., 0., -3.105), patched);
    } else if (scenario == "time") {
      expect(probe->sample(10.01, 0., 0., .03), false);
      expect(probe->sample(10.02, 0., 0., .03), false);
      expect(probe->sample(10.2, 0., 0., .03), patched);
    } else if (scenario == "throttle") {
      probe->throttle(2);
      expect(probe->sample(11., 0., 0., .03), false);
      expect(probe->sample(12., 0., 0., .03), patched);
    } else if (scenario == "pause") {
      probe->pause(true);
      expect(probe->sample(11., 0., 0., .03), false);
      probe->pause(false);
      expect(probe->sample(12., 0., 0., .03), patched);
    } else if (scenario == "translation_x" || scenario == "translation_y") {
      const bool x = scenario == "translation_x";
      expect(probe->sample(11., x ? .008 : 0., x ? 0. : .008, 0.), false);
      expect(probe->sample(12., x ? .009 : 0., x ? 0. : .009, 0.), true);
    } else if (scenario == "heading_boundary") {
      expect(probe->sample(11., 0., 0., .019999), false);
      expect(probe->sample(12., 0., 0., .020001), patched);
    } else if (scenario == "small_combined") {
      expect(probe->sample(11., .004, .004, .015), false);
    } else {
      throw std::runtime_error("unknown scenario");
    }
    std::ifstream maps("/proc/self/maps");
    std::string line;
    while (std::getline(maps, line)) {
      if (line.find("libtoolbox_common.so") != std::string::npos) {
        std::cout << "LOADED " << line << "\n";
        break;
      }
    }
    std::cout << "PASS " << argv[1] << " " << scenario << " decisions=" << checked << "\n";
    probe.reset();
    rclcpp::shutdown();
  } catch (const std::exception &e) {
    std::cerr << "FAIL " << scenario << " " << e.what() << "\n";
    rclcpp::shutdown();
    return 1;
  }
}
