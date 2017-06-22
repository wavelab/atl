#include "atl_core/atl_test.hpp"
#include "atl_core/planning/optimizer.hpp"

namespace atl {

TEST(POpt, test) {
  POpt popt;

  popt.run();
  ASSERT_TRUE(true);
}


}  // end of atl namespace
