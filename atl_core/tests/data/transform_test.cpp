#include "atl/atl_test.hpp"
#include "atl/data/transform.hpp"

namespace atl {

TEST(Transform, nedToNwu) {
  const Vec3 x_ned{1.0, 2.0, 3.0};
  const Vec3 x_nwu = T_nwu_ned * x_ned;
  std::cout << x_nwu << std::endl;

  // EXPECT_EQ(x_ned, T_ned_nwu * x_nwu);
  // EXPECT_EQ(Vec3(1.0, -2.0, -3.0), x_nwu);
}

} // namespace atl
