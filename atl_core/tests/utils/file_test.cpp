#include "atl/utils/file.hpp"
#include "atl/atl_test.hpp"

namespace atl {

TEST(Utils_filesystem, file_exists) {
  EXPECT_TRUE(file_exists("tests/configs/control/position_controller.yaml"));
  EXPECT_FALSE(file_exists("tests/configs/control/bogus.yaml"));
}

TEST(Utils_filesystem, path_split) {
  std::vector<std::string> splits;

  splits = path_split("/a/b/c.yaml");
  EXPECT_EQ(3, splits.size());
  EXPECT_EQ("a", splits[0]);
  EXPECT_EQ("b", splits[1]);
  EXPECT_EQ("c.yaml", splits[2]);
}

TEST(Utils_filesystem, paths_combine) {
  std::string out;

  paths_combine("/a/b/c", "../", out);
  std::cout << out << std::endl;
  EXPECT_EQ("/a/b", out);

  paths_combine("/a/b/c", "../..", out);
  std::cout << out << std::endl;
  EXPECT_EQ("/a", out);

  paths_combine("/a/b/c", "d/e", out);
  std::cout << out << std::endl;
  EXPECT_EQ("/a/b/c/d/e", out);

  paths_combine("./a/b/c", "../d/e", out);
  std::cout << out << std::endl;
  EXPECT_EQ("./a/b/d/e", out);
}

} // end of namespace
