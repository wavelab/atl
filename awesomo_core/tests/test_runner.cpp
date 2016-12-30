#include <gtest/gtest.h>

int main(int argc, char **argv) {
  // supress std::cout
  // std::cout.setstate(std::ios_base::failbit);

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
