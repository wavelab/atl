#include "awesomo_core/awesomo_test.hpp"
#include "awesomo_core/planning/optimizer.hpp"

namespace awesomo {

TEST(POpt, test) {
  POpt popt;

  popt.run();
  ASSERT_TRUE(true);
}

}  // end of awesomo namespace
