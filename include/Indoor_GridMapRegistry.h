#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <optional>
#include <string>
#include <utility>

namespace ligo {
namespace indoor {

struct GridEcefTransform {
  Eigen::Vector3d anchor_ecef_m;
  Eigen::Matrix3d R_ecef_enu;
};

/** Load all * _grid2d.yaml under dir (same layout as grid export / grid_membership_monitor). */
bool loadIndoorGridMapsFromDirectory(const std::string &dir);

void clearIndoorGridMaps();

bool indoorGridMapsLoaded();

/** Rover ECEF [m]. Returns (map_id, source_pcd absolute path) if inside a free/occupied cell (known). */
std::optional<std::pair<std::string, std::string>> lookupIndoorGridKnownPcd(
    const Eigen::Vector3d &ecef_m);

/** Debug helper: returns per-grid membership/classification for the given ECEF input. */
std::string debugIndoorGridLookup(const Eigen::Vector3d &ecef_m);

/** Number of loaded grid maps. */
size_t indoorGridMapCount();

/** Returns (map_id, source_pcd) of the first loaded grid map, or nullopt if none. */
std::optional<std::pair<std::string, std::string>> getFirstGridMapEntry();

/** Returns the ecef_from_enu transform for the given map_id, or nullopt if not found / no transform. */
std::optional<GridEcefTransform> lookupIndoorGridTransformByMapId(const std::string &map_id);

/** Returns the ecef_from_enu transform of the first loaded grid map, or nullopt. */
std::optional<GridEcefTransform> getFirstGridMapTransform();

}  // namespace indoor
}  // namespace ligo
