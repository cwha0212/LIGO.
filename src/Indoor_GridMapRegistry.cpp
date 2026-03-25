#include "Indoor_GridMapRegistry.h"

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <filesystem>
#include <sstream>
#include <unordered_map>
#include <vector>

namespace ligo {
namespace indoor {

namespace {

struct GridMapData {
  std::string map_id;
  std::string yaml_path;
  std::string source_pcd;
  double resolution = 1.0;
  double origin_x = 0.0;
  double origin_y = 0.0;
  std::string frame_id;  // "ecef" | "enu"
  int width = 0;
  int height = 0;
  std::vector<std::uint8_t> pixels;
  bool has_enu_transform = false;
  Eigen::Vector3d anchor_ecef_m = Eigen::Vector3d::Zero();
  Eigen::Matrix3d R_ecef_enu = Eigen::Matrix3d::Identity();
};

std::vector<GridMapData> g_grids;

static std::string trim(std::string s) {
  while (!s.empty() && (s.front() == ' ' || s.front() == '\t')) s.erase(s.begin());
  while (!s.empty() && (s.back() == ' ' || s.back() == '\t')) s.pop_back();
  return s;
}

static std::unordered_map<std::string, std::string> parseSimpleYaml(const std::string &path) {
  std::unordered_map<std::string, std::string> result;
  std::ifstream f(path);
  std::string line;
  while (std::getline(f, line)) {
    line = trim(line);
    if (line.empty() || line[0] == '#')
      continue;
    const auto colon = line.find(':');
    if (colon == std::string::npos)
      continue;
    std::string key = trim(line.substr(0, colon));
    std::string val = trim(line.substr(colon + 1));
    result[key] = val;
  }
  return result;
}

static bool parseOriginTriplet(const std::string &text, double &ox, double &oy, double &oz) {
  std::string s = trim(text);
  if (s.size() < 5 || s.front() != '[' || s.back() != ']')
    return false;
  s = s.substr(1, s.size() - 2);
  std::stringstream ss(s);
  std::string part;
  std::vector<double> nums;
  while (std::getline(ss, part, ',')) {
    part = trim(part);
    if (part.empty())
      continue;
    try {
      nums.push_back(std::stod(part));
    } catch (...) {
      return false;
    }
  }
  if (nums.size() != 3)
    return false;
  ox = nums[0];
  oy = nums[1];
  oz = nums[2];
  return true;
}

static bool readPgmP5(const std::string &pgm_path, int &out_w, int &out_h, std::vector<std::uint8_t> &out_px) {
  std::ifstream f(pgm_path, std::ios::binary);
  if (!f)
    return false;
  std::string magic;
  if (!(f >> magic) || magic != "P5")
    return false;
  auto skip_comments = [&f]() {
    while (f.peek() == '#' || f.peek() == '\n' || f.peek() == ' ' || f.peek() == '\r') {
      if (f.peek() == '#') {
        std::string dummy;
        std::getline(f, dummy);
      } else
        f.get();
    }
  };
  skip_comments();
  int w = 0, h = 0, maxval = 0;
  if (!(f >> w >> h >> maxval))
    return false;
  f.get();
  if (maxval != 255)
    return false;
  out_px.resize(static_cast<size_t>(w) * static_cast<size_t>(h));
  f.read(reinterpret_cast<char *>(out_px.data()), out_px.size());
  if (static_cast<std::size_t>(f.gcount()) != out_px.size())
    return false;
  out_w = w;
  out_h = h;
  return true;
}

static std::optional<std::pair<double, double>> ecefToMapXY(const GridMapData &g, double x, double y, double z) {
  std::string frame = g.frame_id;
  for (auto &c : frame) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  if (frame == "ecef")
    return std::make_pair(x, y);
  if (frame == "enu") {
    if (!g.has_enu_transform)
      return std::nullopt;
    const Eigen::Vector3d d(x - g.anchor_ecef_m.x(), y - g.anchor_ecef_m.y(), z - g.anchor_ecef_m.z());
    const Eigen::Vector3d enu = g.R_ecef_enu.transpose() * d;
    return std::make_pair(enu.x(), enu.y());
  }
  return std::nullopt;
}

static std::pair<bool, std::string> classify(const GridMapData &g, double x, double y, double z) {
  auto map_xy = ecefToMapXY(g, x, y, z);
  if (!map_xy)
    return {false, "outside"};
  const double mx = map_xy->first;
  const double my = map_xy->second;
  const int col = static_cast<int>(std::floor((mx - g.origin_x) / g.resolution));
  const int row_from_bottom = static_cast<int>(std::floor((my - g.origin_y) / g.resolution));
  const int row = g.height - 1 - row_from_bottom;
  if (row < 0 || row >= g.height || col < 0 || col >= g.width)
    return {false, "outside"};
  const std::uint8_t v = g.pixels[static_cast<size_t>(row) * static_cast<size_t>(g.width) + static_cast<size_t>(col)];
  if (v <= 50)
    return {true, "occupied"};
  if (v >= 250)
    return {true, "free"};
  return {true, "unknown"};
}

}  // namespace

bool loadIndoorGridMapsFromDirectory(const std::string &dir) {
  clearIndoorGridMaps();
  std::error_code ec;
  if (dir.empty() || !std::filesystem::is_directory(dir, ec))
    return false;

  std::vector<std::filesystem::path> yaml_paths;
  for (const auto &e : std::filesystem::directory_iterator(dir, ec)) {
    if (!e.is_regular_file())
      continue;
    const auto name = e.path().filename().string();
    const std::string suffix = "_grid2d.yaml";
    if (name.size() > suffix.size() && name.compare(name.size() - suffix.size(), suffix.size(), suffix) == 0)
      yaml_paths.push_back(e.path());
  }
  std::sort(yaml_paths.begin(), yaml_paths.end());

  namespace fs = std::filesystem;
  for (const auto &yp : yaml_paths) {
    const std::string ypath = yp.string();
    const auto kv = parseSimpleYaml(ypath);
    auto it_img = kv.find("image");
    if (it_img == kv.end() || it_img->second.empty()) {
      RCLCPP_WARN(rclcpp::get_logger("ligo"), "[indoor/grid] skip yaml (no image): %s", ypath.c_str());
      continue;
    }
    fs::path pgm_path = it_img->second;
    if (pgm_path.is_relative())
      pgm_path = yp.parent_path() / pgm_path;
    if (!fs::exists(pgm_path)) {
      RCLCPP_WARN(rclcpp::get_logger("ligo"), "[indoor/grid] skip (pgm missing): %s", pgm_path.string().c_str());
      continue;
    }

    GridMapData g;
    g.yaml_path = ypath;
    {
      std::string stem = yp.stem().string();
      const std::string suf = "_grid2d";
      if (stem.size() >= suf.size() && stem.compare(stem.size() - suf.size(), suf.size(), suf) == 0)
        g.map_id = stem.substr(0, stem.size() - suf.size());
      else
        g.map_id = stem;
    }
    auto it_pcd = kv.find("source_pcd");
    if (it_pcd == kv.end() || it_pcd->second.empty()) {
      RCLCPP_WARN(rclcpp::get_logger("ligo"), "[indoor/grid] skip (no source_pcd): %s", ypath.c_str());
      continue;
    }
    fs::path pcd_path = it_pcd->second;
    if (pcd_path.is_relative())
      pcd_path = yp.parent_path() / pcd_path;
    g.source_pcd = fs::weakly_canonical(pcd_path, ec).string();
    if (!fs::exists(g.source_pcd)) {
      RCLCPP_WARN(rclcpp::get_logger("ligo"), "[indoor/grid] skip (pcd missing): %s", g.source_pcd.c_str());
      continue;
    }

    auto it_res = kv.find("resolution");
    g.resolution = it_res != kv.end() && !it_res->second.empty() ? std::stod(it_res->second) : 1.0;
    auto it_origin = kv.find("origin");
    double origin_z_ignored = 0.0;
    if (it_origin == kv.end() || !parseOriginTriplet(it_origin->second, g.origin_x, g.origin_y, origin_z_ignored)) {
      RCLCPP_WARN(rclcpp::get_logger("ligo"), "[indoor/grid] skip (bad origin): %s", ypath.c_str());
      continue;
    }
    auto it_frame = kv.find("frame_id");
    g.frame_id = it_frame != kv.end() ? it_frame->second : "ecef";

    auto it_anchor = kv.find("anchor_ecef_m");
    auto it_R = kv.find("R_ecef_enu_row_major");
    if (it_anchor != kv.end() && it_R != kv.end()) {
      double ax, ay, az;
      if (parseOriginTriplet(it_anchor->second, ax, ay, az)) {
        g.anchor_ecef_m = Eigen::Vector3d(ax, ay, az);
        std::string Rtxt = it_R->second;
        Rtxt = trim(Rtxt);
        if (Rtxt.size() >= 2 && Rtxt.front() == '[' && Rtxt.back() == ']') {
          Rtxt = Rtxt.substr(1, Rtxt.size() - 2);
          std::stringstream ss(Rtxt);
          std::string part;
          std::vector<double> coefs;
          while (std::getline(ss, part, ',')) {
            part = trim(part);
            if (part.empty())
              continue;
            coefs.push_back(std::stod(part));
          }
          if (coefs.size() == 9) {
            g.R_ecef_enu <<
                coefs[0], coefs[1], coefs[2],
                coefs[3], coefs[4], coefs[5],
                coefs[6], coefs[7], coefs[8];
            g.has_enu_transform = true;
          }
        }
      }
    }

    if (!readPgmP5(pgm_path.string(), g.width, g.height, g.pixels)) {
      RCLCPP_WARN(rclcpp::get_logger("ligo"), "[indoor/grid] skip (pgm read failed): %s", pgm_path.string().c_str());
      continue;
    }

    g_grids.push_back(std::move(g));
    RCLCPP_INFO(rclcpp::get_logger("ligo"), "[indoor/grid] loaded map_id=%s pcd=%s",
                g_grids.back().map_id.c_str(), g_grids.back().source_pcd.c_str());
  }

  return !g_grids.empty();
}

void clearIndoorGridMaps() {
  g_grids.clear();
}

bool indoorGridMapsLoaded() {
  return !g_grids.empty();
}

size_t indoorGridMapCount() {
  return g_grids.size();
}

std::optional<std::pair<std::string, std::string>> getFirstGridMapEntry() {
  if (g_grids.empty()) return std::nullopt;
  return std::make_pair(g_grids.front().map_id, g_grids.front().source_pcd);
}

std::optional<std::pair<std::string, std::string>> lookupIndoorGridKnownPcd(const Eigen::Vector3d &ecef_m) {
  const double x = ecef_m.x(), y = ecef_m.y(), z = ecef_m.z();
  for (const auto &g : g_grids) {
    auto [inside, state] = classify(g, x, y, z);
    if (inside && (state == "free" || state == "occupied"))
      return std::make_pair(g.map_id, g.source_pcd);
  }
  return std::nullopt;
}

std::optional<GridEcefTransform> lookupIndoorGridTransformByMapId(const std::string &map_id) {
  for (const auto &g : g_grids) {
    if (g.map_id == map_id && g.has_enu_transform)
      return GridEcefTransform{g.anchor_ecef_m, g.R_ecef_enu};
  }
  return std::nullopt;
}

std::optional<GridEcefTransform> getFirstGridMapTransform() {
  for (const auto &g : g_grids) {
    if (g.has_enu_transform)
      return GridEcefTransform{g.anchor_ecef_m, g.R_ecef_enu};
  }
  return std::nullopt;
}

}  // namespace indoor
}  // namespace ligo
