#pragma once

#include <Eigen/Core>
#include <optional>
#include <string>
#include <utility>

namespace ligo {
namespace indoor {

/** Load all * _grid2d.yaml under dir (same layout as grid export / grid_membership_monitor). */
bool loadIndoorGridMapsFromDirectory(const std::string &dir);

void clearIndoorGridMaps();

bool indoorGridMapsLoaded();

/** Rover ECEF [m]. Returns (map_id, source_pcd absolute path) if inside a free/occupied cell (known). */
std::optional<std::pair<std::string, std::string>> lookupIndoorGridKnownPcd(
    const Eigen::Vector3d &ecef_m);

/** Number of loaded grid maps. */
size_t indoorGridMapCount();

/** Returns (map_id, source_pcd) of the first loaded grid map, or nullopt if none. */
std::optional<std::pair<std::string, std::string>> getFirstGridMapEntry();

}  // namespace indoor
}  // namespace ligo
