#include <gtest/gtest.h>

#include "awesomo_core/utils/config.hpp"

#define TEST_CONFIG "tests/configs/config.yaml"


TEST(ConfigParam, constructor) {
  awesomo::ConfigParam param;

  ASSERT_EQ(awesomo::TYPE_NOT_SET, param.type);
  ASSERT_EQ(awesomo::CLASS_NOT_SET, param.type_class);
  ASSERT_EQ("", param.key);
  ASSERT_EQ(false, param.optional);

  ASSERT_EQ(NULL, param.b);
  ASSERT_EQ(NULL, param.i);
  ASSERT_EQ(NULL, param.f);
  ASSERT_EQ(NULL, param.d);
  ASSERT_EQ(NULL, param.s);

  ASSERT_EQ(NULL, param.b_array);
  ASSERT_EQ(NULL, param.i_array);
  ASSERT_EQ(NULL, param.f_array);
  ASSERT_EQ(NULL, param.d_array);
  ASSERT_EQ(NULL, param.s_array);

  ASSERT_EQ(NULL, param.vec2);
  ASSERT_EQ(NULL, param.vec3);
  ASSERT_EQ(NULL, param.vec4);
  ASSERT_EQ(NULL, param.vecx);

  ASSERT_EQ(NULL, param.mat2);
  ASSERT_EQ(NULL, param.mat3);
  ASSERT_EQ(NULL, param.mat4);
  ASSERT_EQ(NULL, param.matx);
}

TEST(ConfigParser, constructor) {
  awesomo::ConfigParser parser;

  ASSERT_FALSE(parser.configured);
  ASSERT_FALSE(parser.loaded);
}

TEST(ConfigParser, addParam) {
  int i;
  awesomo::ConfigParser parser;

  parser.addParam(awesomo::INT, "integer", &i);

  ASSERT_EQ(1, parser.params.size());
  ASSERT_EQ(awesomo::INT, parser.params[0].type);
  ASSERT_EQ("integer", parser.params[0].key);
  ASSERT_TRUE(parser.params[0].i != NULL);
}

TEST(ConfigParser, loadPrimitive) {
  int i;
  float f;
  double d;
  std::string s;
  awesomo::ConfigParser parser;
  awesomo::ConfigParam param;

  // setup
  parser.root = YAML::LoadFile(TEST_CONFIG);

  // INTEGER
  param.optional = false;
  param.type = awesomo::INT;
  param.key = "integer";
  param.i = &i;
  parser.loadPrimitive(param);
  ASSERT_EQ(1, i);

  // FLOAT
  param.optional = false;
  param.type = awesomo::FLOAT;
  param.key = "float";
  param.f = &f;
  parser.loadPrimitive(param);
  ASSERT_FLOAT_EQ(2.0, f);

  // DOUBLE
  param.optional = false;
  param.type = awesomo::DOUBLE;
  param.key = "double";
  param.d = &d;
  parser.loadPrimitive(param);
  ASSERT_FLOAT_EQ(3.0, d);

  // STRING
  param.optional = false;
  param.type = awesomo::STRING;
  param.key = "string";
  param.s = &s;
  parser.loadPrimitive(param);
  ASSERT_EQ("hello world!", s);
}

TEST(ConfigParser, loadArray) {
  std::vector<bool> b_array;
  std::vector<int> i_array;
  std::vector<float> f_array;
  std::vector<double> d_array;
  std::vector<std::string> s_array;
  awesomo::ConfigParser parser;
  awesomo::ConfigParam param;

  // setup
  parser.root = YAML::LoadFile(TEST_CONFIG);

  // BOOL ARRAY
  param.optional = false;
  param.type = awesomo::BOOL_ARRAY;
  param.key = "bool_array";
  param.b_array = &b_array;
  parser.loadArray(param);

  ASSERT_TRUE(b_array[0]);
  ASSERT_FALSE(b_array[1]);
  ASSERT_TRUE(b_array[2]);
  ASSERT_FALSE(b_array[3]);

  // INTEGER
  param.optional = false;
  param.type = awesomo::INT_ARRAY;
  param.key = "integer_array";
  param.i_array = &i_array;
  parser.loadArray(param);

  for (int i = 0; i < 4; i++) {
    ASSERT_EQ(i + 1, i_array[i]);
  }

  // FLOAT
  param.optional = false;
  param.type = awesomo::FLOAT_ARRAY;
  param.key = "float_array";
  param.f_array = &f_array;
  parser.loadArray(param);

  for (int i = 0; i < 4; i++) {
    ASSERT_FLOAT_EQ((float) i + 1.0, f_array[i]);
  }

  // DOUBLE
  param.optional = false;
  param.type = awesomo::DOUBLE_ARRAY;
  param.key = "double_array";
  param.d_array = &d_array;
  parser.loadArray(param);

  for (int i = 0; i < 4; i++) {
    ASSERT_FLOAT_EQ((double) i + 1.0, d_array[i]);
  }

  // STRING
  param.optional = false;
  param.type = awesomo::STRING_ARRAY;
  param.key = "string_array";
  param.s_array = &s_array;
  parser.loadArray(param);

  ASSERT_EQ("1.0", s_array[0]);
  ASSERT_EQ("2.0", s_array[1]);
  ASSERT_EQ("3.0", s_array[2]);
  ASSERT_EQ("4.0", s_array[3]);
}

TEST(ConfigParser, loadVector) {
  awesomo::Vec2 vec2;
  awesomo::Vec3 vec3;
  awesomo::Vec4 vec4;
  awesomo::VecX vecx;
  awesomo::ConfigParser parser;
  awesomo::ConfigParam param;

  // setup
  parser.root = YAML::LoadFile(TEST_CONFIG);

  // VECTOR 2
  param.optional = false;
  param.type = awesomo::VEC2;
  param.key = "vector2";
  param.vec2 = &vec2;
  parser.loadVector(param);

  ASSERT_FLOAT_EQ(1.0, vec2(0));
  ASSERT_FLOAT_EQ(2.0, vec2(1));

  // VECTOR 3
  param.optional = false;
  param.type = awesomo::VEC3;
  param.key = "vector3";
  param.vec3 = &vec3;
  parser.loadVector(param);

  ASSERT_FLOAT_EQ(1.0, vec3(0));
  ASSERT_FLOAT_EQ(2.0, vec3(1));
  ASSERT_FLOAT_EQ(3.0, vec3(2));

  // VECTOR 4
  param.optional = false;
  param.type = awesomo::VEC4;
  param.key = "vector4";
  param.vec4 = &vec4;
  parser.loadVector(param);

  ASSERT_FLOAT_EQ(1.0, vec4(0));
  ASSERT_FLOAT_EQ(2.0, vec4(1));
  ASSERT_FLOAT_EQ(3.0, vec4(2));
  ASSERT_FLOAT_EQ(4.0, vec4(3));

  // VECTOR X
  param.optional = false;
  param.type = awesomo::VECX;
  param.key = "vector";
  param.vecx = &vecx;
  parser.loadVector(param);

  for (int i = 0; i < 10; i++) {
    ASSERT_FLOAT_EQ((double) i + 1.0, vecx(i));
  }
}

TEST(ConfigParser, loadMatrix) {
  int index;
  awesomo::Mat2 mat2;
  awesomo::Mat3 mat3;
  awesomo::Mat4 mat4;
  awesomo::MatX matx;
  awesomo::ConfigParser parser;
  awesomo::ConfigParam param;

  // setup
  parser.root = YAML::LoadFile(TEST_CONFIG);

  // MATRIX 2
  param.optional = false;
  param.type = awesomo::MAT2;
  param.key = "matrix2";
  param.mat2 = &mat2;
  parser.loadMatrix(param);

  ASSERT_FLOAT_EQ(1.0, mat2(0, 0));
  ASSERT_FLOAT_EQ(2.0, mat2(0, 1));
  ASSERT_FLOAT_EQ(3.0, mat2(1, 0));
  ASSERT_FLOAT_EQ(4.0, mat2(1, 1));

  // MATRIX 3
  param.optional = false;
  param.type = awesomo::MAT3;
  param.key = "matrix3";
  param.mat3 = &mat3;
  parser.loadMatrix(param);

  index = 0;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      ASSERT_FLOAT_EQ(index + 1.0, mat3(i, j));
      index++;
    }
  }

  // MATRIX 4
  param.optional = false;
  param.type = awesomo::MAT4;
  param.key = "matrix4";
  param.mat4 = &mat4;
  parser.loadMatrix(param);

  index = 0;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      ASSERT_FLOAT_EQ(index + 1.0, mat4(i, j));
      index++;
    }
  }

  // MATRIX X
  param.optional = false;
  param.type = awesomo::MATX;
  param.key = "matrix";
  param.matx = &matx;
  parser.loadMatrix(param);

  index = 0;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 4; j++) {
      ASSERT_FLOAT_EQ(index + 1.0, matx(i, j));
      index++;
    }
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
