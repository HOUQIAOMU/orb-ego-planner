#pragma once

#include <filesystem>
#include <string>
#include <vector>

namespace ego_occupancy {

struct Options {
  std::filesystem::path vocabulary;
  std::filesystem::path camera;
  std::filesystem::path map_config;
  std::filesystem::path output;
  std::filesystem::path dataset;
  std::filesystem::path lock_root;
  std::string serial = "042222071908";
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
