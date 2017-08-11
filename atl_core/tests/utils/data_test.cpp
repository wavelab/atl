#include "atl/utils/data.hpp"
#include "atl/atl_test.hpp"

#define TEST_DATA "tests/data/utils/matrix.dat"
#define TEST_OUTPUT "/tmp/matrix.dat"

namespace atl {

TEST(Utils_data, csvrows) {
  int rows;
  rows = csvrows(TEST_DATA);
  EXPECT_EQ(281, rows);
}

TEST(Utils_data, csvcols) {
  int cols;
  cols = csvcols(TEST_DATA);
  EXPECT_EQ(2, cols);
}

TEST(Utils_data, csv2mat) {
  MatX data;

  csv2mat(TEST_DATA, true, data);
  EXPECT_EQ(280, data.rows());
  EXPECT_EQ(2, data.cols());
  EXPECT_FLOAT_EQ(-2.22482078596, data(0, 0));
  EXPECT_FLOAT_EQ(9.9625789766, data(0, 1));
  EXPECT_FLOAT_EQ(47.0485650525, data(279, 0));
  EXPECT_FLOAT_EQ(613.503760567, data(279, 1));
}

TEST(Utils_data, mat2csv) {
  MatX x;
  MatX y;

  csv2mat(TEST_DATA, true, x);
  mat2csv(TEST_OUTPUT, x);
  csv2mat(TEST_OUTPUT, false, y);

  for (int i = 0; i < x.rows(); i++) {
    for (int j = 0; j < x.cols(); j++) {
      ASSERT_NEAR(x(i, j), y(i, j), 0.1);
    }
  }
}

} // namespace atl
