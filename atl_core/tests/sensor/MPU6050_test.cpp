#include "atl_core/atl_test.hpp"
#include "atl_core/sensor/MPU6050.hpp"

namespace atl {

TEST(MPU6050, configure) {
  int retval;
  MPU6050 imu;

  retval = imu.configure();
  ASSERT_EQ(0.0, retval);
}

}  // end of atl namespace
