#include "atl/sensor/MPU6050.hpp"
#include "atl/atl_test.hpp"

namespace atl {

TEST(MPU6050, configure) {
  int retval;
  MPU6050 imu;

  retval = imu.configure();
  EXPECT_EQ(0.0, retval);
}

}  // namespace atl
