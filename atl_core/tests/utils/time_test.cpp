#include "atl_core/atl_test.hpp"
#include "atl_core/utils/time.hpp"


namespace atl {

TEST(Utils_time, ticAndtoc) {
  struct timespec start;

  tic(&start);
  usleep(10 * 1000);
  ASSERT_TRUE(toc(&start) < 0.011);
  ASSERT_TRUE(toc(&start) > 0.009);
  ASSERT_TRUE(mtoc(&start) < 11.0);
  ASSERT_TRUE(mtoc(&start) > 9.0);
}

}  // end of atl namespace
