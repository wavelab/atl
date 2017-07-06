#include "atl/planning/optimizer.hpp"
#include "atl/atl_test.hpp"

namespace atl {

TEST(POpt, test) {
  POpt popt;

  popt.run();
  EXPECT_TRUE(true);
}

}  // namespace atl
