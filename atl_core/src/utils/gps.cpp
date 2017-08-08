#include "atl/utils/gps.hpp"

namespace atl {

void latlon_offset(
  double lat_ref,
  double lon_ref,
  double offset_N,
  double offset_E,
  double *lat_new,
  double *lon_new) {
  *lat_new = lat_ref + (offset_E / EARTH_RADIUS_M);
  *lon_new = lon_ref + (offset_N / EARTH_RADIUS_M) / cos(deg2rad(lat_ref));
}

void latlon_diff(
  double lat_ref,
  double lon_ref,
  double lat,
  double lon,
  double *dist_N,
  double *dist_E) {
  double d_lon = lon - lon_ref;
  double d_lat = lat - lat_ref;

  *dist_N = deg2rad(d_lat) * EARTH_RADIUS_M;
  *dist_E = deg2rad(d_lon) * EARTH_RADIUS_M * cos(deg2rad(lat));
}

double latlon_dist(double lat_ref, double lon_ref, double lat, double lon) {
  double dist_N = 0.0;
  double dist_E = 0.0;

  latlon_diff(lat_ref, lon_ref, lat, lon, &dist_N, &dist_E);
  double dist = sqrt(pow(dist_N, 2) + pow(dist_E, 2));

  return dist;
}

}  // namepsace atl
