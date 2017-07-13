#include "atl/atl_test.hpp"
#include "atl/utils/gps.hpp"

namespace atl {

TEST(Utils, latlon_offset) {
  // UWaterloo 110 yards Canadian Football field from one end to another
  double lat = 43.474357;
  double lon = -80.550415;

  double offset_N = 44.1938;
  double offset_E = 90.2336;

  double lat_new = 0.0;
  double lon_new = 0.0;

  // calculate football field GPS coordinates
  latlon_offset(lat, lon, offset_N, offset_E, &lat_new, &lon_new);
  std::cout << "lat new: " << lat_new << std::endl;
  std::cout << "lon new: " << lon_new << std::endl;

  // gps coordinates should be close to (43.474754, -80.549298)
  EXPECT_NEAR(43.474754, lat_new, 0.0015);
  EXPECT_NEAR(-80.549298, lon_new, 0.0015);
}

TEST(Utils, latlon_diff) {
  // UWaterloo 110 yards Canadian Football field from one end to another
  double lat_ref = 43.474357;
  double lon_ref = -80.550415;
  double lat = 43.474754;
  double lon = -80.549298;

  double dist_N = 0.0;
  double dist_E = 0.0;

  // calculate football field distance
  latlon_diff(lat_ref, lon_ref, lat, lon, &dist_N, &dist_E);
  double dist = sqrt(pow(dist_N, 2) + pow(dist_E, 2));
  std::cout << "distance north: " << dist_N << std::endl;
  std::cout << "distance east: " << dist_E << std::endl;

  // 110 yards is approx 100 meters
  EXPECT_NEAR(100, dist, 1.0);
}

TEST(Utils, latlon_dist) {
  // UWaterloo 110 yards Canadian Football field from one end to another
  double lat_ref = 43.474357;
  double lon_ref = -80.550415;
  double lat = 43.474754;
  double lon = -80.549298;

  // calculate football field distance
  double dist = latlon_dist(lat_ref, lon_ref, lat, lon);
  std::cout << "distance: " << dist << std::endl;

  // 110 yards is approx 100 meters
  EXPECT_NEAR(100, dist, 1.0);
}

}  // end of namespace
