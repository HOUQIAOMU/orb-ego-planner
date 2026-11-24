#pragma once

#include <filesystem>
#include <string>
#include <vector>

namespace ego_occupancy {

enum class CameraBackend { RealSense, Orbbec };

struct Options {
  std::filesystem::path planning_config;
  double probe_start_m = 0.5;
  double probe_goal_m = 0.9;
  std::filesystem::path vocabulary;
  std::filesystem::path camera;
  std::filesystem::path map_config;
  std::filesystem::path output;
  std::filesystem::path dataset;
  std::filesystem::path lock_root;
  CameraBackend camera_backend = CameraBackend::RealSense;
  std::string serial;
  std::string display_color = "depth";
  bool live = false;
  bool check = false;
  bool viewer = true;
  bool help = false;
  int max_frames = 0;
  int stride = 2;
  int warmup = 30;
  double max_seconds = 120.0;
  double mapping_hz = 10.0;
};

Options parseOptions(const std::vector<std::string>& arguments);
std::string usage();

}  // namespace ego_occupancy
