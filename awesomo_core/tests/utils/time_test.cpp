#include <gtest/gtest.h>

#include "awesomo_core/utils/time.hpp"


namespace awesomo {

TEST(Utils, ticAndtoc) {
  struct timespec start;

  tic(&start);
  usleep(10 * 1000);
  std::cout << toc(&start) << std::endl;
  std::cout << mtoc(&start) << std::endl;
  ASSERT_TRUE(toc(&start) < 0.011);
  ASSERT_TRUE(toc(&start) > 0.009);
  ASSERT_TRUE(mtoc(&start) < 11.0);
  ASSERT_TRUE(mtoc(&start) > 9.0);
}

}  // end of awesomo namespace
