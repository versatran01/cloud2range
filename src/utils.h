#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace cloud2range {

static constexpr double PointAltitude(double x, double y, double z) {
  // [-pi, pi]
  return std::atan2(z, std::hypot(x, y));
}

/**
 * @brief Compute altitude of a point in radius, assumes point in local frame
 * @details Altitude is defined as the angle between the xy-plane and the point
 * https://en.wikipedia.org/wiki/Horizontal_coordinate_system
 * @param point
 */
template <typename PointT>
double PointAltitude(const PointT &point) {
  return PointAltitude(point.x, point.y, point.z);
}

static constexpr double PointAzimuth(double x, double y) {
  const auto a = std::atan2(y, x);
  // Convert to [0, 2pi)
  return y >= 0 ? a : a + M_PI * 2;
}

/**
 * @brief Compute azimuth of a point in radius, assumes point in local frame
 * @details Azimuth is defined as the angle between the projection of the point
 * and the north, https://en.wikipedia.org/wiki/Horizontal_coordinate_system
 * @param point
 */
template <typename PointT>
double PointAzimuth(const PointT &point) {
  return PointAzimuth(point.x, point.y);
}

static constexpr double PointRange(double x, double y, double z) {
  return std::sqrt(x * x + y * y + z * z);
}

template <typename PointT>
double PointRange(const PointT &point) {
  return PointRange(point.x, point.y, point.z);
}

/// Degree from Radian
static constexpr double Deg_Rad(double rad) { return rad * 180 / M_PI; }

/// Radian from Degree
static constexpr double Rad_Deg(double deg) { return deg / 180 * M_PI; }
}  // namespace cloud2range
