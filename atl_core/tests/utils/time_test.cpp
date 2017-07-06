#include "atl/utils/time.hpp"
#include "atl/atl_test.hpp"

namespace atl {

TEST(Utils_time, ticAndtoc) {
  struct timespec start;

  tic(&start);
  usleep(10 * 1000);
  EXPECT_TRUE(toc(&start) < 0.011);
  EXPECT_TRUE(toc(&start) > 0.009);
  EXPECT_TRUE(mtoc(&start) < 11.0);
  EXPECT_TRUE(mtoc(&start) > 9.0);
}

}  // namespace atl
