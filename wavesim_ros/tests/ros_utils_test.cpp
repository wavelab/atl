#include <gtest/gtest.h>

#include "wavesim_ros/utils/utils.hpp"

namespace wavesim {
namespace ros {

using namespace wave;

TEST(ROSUtils, ros2gaz) {
  // test vector version of ros2gaz
  Vec3 ros_pos, gaz_pos;

  ros_pos << 1.0, 2.0, 3.0;
  ros2gaz(ros_pos, gaz_pos);

  std::cout << "ros pos: " << ros_pos.transpose() << std::endl;
  std::cout << "gaz pos: " << gaz_pos.transpose() << std::endl;

  ASSERT_FLOAT_EQ(1.0, -gaz_pos(1));
  ASSERT_FLOAT_EQ(2.0, gaz_pos(0));
  ASSERT_FLOAT_EQ(3.0, gaz_pos(2));

  // test quaternion version of ros2gaz
  Vec3 ros_euler, gaz_euler;
  Quaternion ros_quat, gaz_quat;

  ros_euler << deg2rad(0.0), deg2rad(0.0), deg2rad(90.0);
  euler2quat(ros_euler, 321, ros_quat);

  ros2gaz(ros_quat, gaz_quat);
  quat2euler(gaz_quat, 321, gaz_euler);

  std::cout << "ros euler: " << ros_euler.transpose() << std::endl;
  std::cout << "gaz euler: " << gaz_euler.transpose() << std::endl;

  // ASSERT_FLOAT_EQ(1.0, ros_pos(0));
  // ASSERT_FLOAT_EQ(2.0, ros_pos(1));
  // ASSERT_FLOAT_EQ(3.0, ros_pos(2));
}

TEST(ROSUtils, gaz2ros) {
  // test vector version of gaz2ros
  Vec3 gaz_pos, ros_pos;
  gaz_pos << 1.0, 2.0, 3.0;
  gaz2ros(gaz_pos, ros_pos);

  std::cout << "gaz pos: " << gaz_pos.transpose() << std::endl;
  std::cout << "ros pos: " << ros_pos.transpose() << std::endl;

  ASSERT_FLOAT_EQ(1.0, ros_pos(1));
  ASSERT_FLOAT_EQ(2.0, -ros_pos(0));
  ASSERT_FLOAT_EQ(3.0, ros_pos(2));


  // test quaternion version of gaz2ros
  Vec3 ros_euler, gaz_euler;
  Quaternion gaz_quat, ros_quat;

  gaz_euler << deg2rad(10.0), deg2rad(0.0), deg2rad(90.0);
  euler2quat(gaz_euler, 321, gaz_quat);

  gaz2ros(gaz_quat, ros_quat);
  quat2euler(ros_quat, 321, ros_euler);

  std::cout << "gaz euler: " << gaz_euler.transpose() << std::endl;
  std::cout << "ros euler: " << ros_euler.transpose() << std::endl;

  gaz2ros(gaz_quat, ros_quat);
  quat2euler(ros_quat, 321, ros_euler);
}

}  // end of ros namespace
}  // end of wavesim  namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
