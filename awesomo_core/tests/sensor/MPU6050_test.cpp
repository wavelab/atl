#include "awesomo_core/awesomo_test.hpp"
#include "awesomo_core/sensor/MPU6050.hpp"

namespace awesomo {

TEST(MPU6050, configure) {
  int retval;
  MPU6050 imu;

  retval = imu.configure();
  ASSERT_EQ(0.0, retval);
}

}  // end of awesomo namespace
