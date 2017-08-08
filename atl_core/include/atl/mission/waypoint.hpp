#ifndef ATL_MISSION_WAYPOINT_HPP
#define ATL_MISSION_WAYPOINT_HPP

#include "atl/utils/utils.hpp"

namespace atl {

/**
 * Waypoint
 */
class Waypoint {
public:
  double latitude;
  double longitude;
  double altitude;
  double staytime;
  double heading;

  Waypoint()
      : latitude{0.0},
        longitude{0.0},
        altitude{0.0},
        staytime{0.0},
        heading{0.0} {}
  Waypoint(double latitude, double longitude)
      : latitude{latitude}, longitude{longitude} {}
  Waypoint(
    double latitude,
    double longitude,
    double altitude,
    double staytime,
    double heading)
      : latitude{latitude},
        longitude{longitude},
        altitude{altitude},
        staytime{staytime},
        heading{heading} {}

  /**
   * Calculate distance away from another waypoint
   * @param wp 2nd Waypoint to calculate distance away from
   * @return Distance between this waypoint and waypoint `wp`
   */
  double distance(const Waypoint &wp) {
    return latlon_dist(latitude, longitude, wp.latitude, wp.longitude);
  }

  friend std::ostream &operator<<(std::ostream &out, const Waypoint &wp) {
    out << "latitude: " << wp.latitude << std::endl;
    out << "longitude: " << wp.longitude << std::endl;
    out << "altitude: " << wp.altitude << std::endl;
    out << "staytime: " << wp.staytime << std::endl;
    out << "heading: " << wp.heading;
    return out;
  }
};

}  // namespace atl

#endif
