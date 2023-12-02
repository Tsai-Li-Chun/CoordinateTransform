#include "pch.h"

#include <iostream>
#include <cstdlib> /* 亂數相關函數 */
#include <ctime>   /* 時間相關函數 */

#include "MathTool.h"

#define UNIT_TEST_PI (atan(1) * 4)
#define UNIT_TEST_RAD_TO_DEG (1.0 / UNIT_TEST_PI * 180.0)
#define UNIT_TEST_DEG_TO_RAD (1.0 / UNIT_TEST_RAD_TO_DEG)

#define UNIT_TEST_PRECISION (1e-7)

MathTool math_tool;

// 產生指定範圍 [min , max] 的浮點數亂數
double inline produce_random_value_in_range(const double max, const double min) {
	srand(time(NULL)); // 以時間為基礎的亂數種子
	// rand() : [0 , RAND_MAX]
	// rand() / RAND_MAX : 產生範圍 [0 , 1] 的浮點數亂數
	return (max - min) * (double) rand() / RAND_MAX + min;
}
double inline  produce_random_value() {
	return produce_random_value_in_range(0.5 * RAND_MAX, -0.5 * RAND_MAX);
}

TEST(MathTool, sind) {
	double rand_angle_deg = produce_random_value();
	double rand_angle_rad = rand_angle_deg * UNIT_TEST_DEG_TO_RAD;

	EXPECT_NEAR(sin(rand_angle_rad), math_tool.sind(rand_angle_deg), UNIT_TEST_PRECISION);
}
TEST(MathTool, cosd) {
	double rand_angle_deg = produce_random_value();
	double rand_angle_rad = rand_angle_deg * UNIT_TEST_DEG_TO_RAD;

	EXPECT_NEAR(cos(rand_angle_rad), math_tool.cosd(rand_angle_deg), UNIT_TEST_PRECISION);
}
TEST(MathTool, tand) {
	double rand_angle_deg = produce_random_value();
	double rand_angle_rad = rand_angle_deg * UNIT_TEST_DEG_TO_RAD;

	EXPECT_NEAR(tan(rand_angle_rad), math_tool.tand(rand_angle_deg), UNIT_TEST_PRECISION);
}
TEST(MathTool, asind) {
	double value = produce_random_value_in_range(1.0, -1.0);

	EXPECT_NEAR(asin(value), math_tool.asind(value) * UNIT_TEST_DEG_TO_RAD, UNIT_TEST_PRECISION);
}
TEST(MathTool, acosd) {
	double value = produce_random_value_in_range(1.0, -1.0);

	EXPECT_NEAR(acos(value), math_tool.acosd(value) * UNIT_TEST_DEG_TO_RAD, UNIT_TEST_PRECISION);
}
TEST(MathTool, atan2d) {
	double numerator   = produce_random_value(); // 分子
	double denominator = produce_random_value(); // 分母

	EXPECT_NEAR(atan2(numerator, denominator), math_tool.atan2d(numerator, denominator) * UNIT_TEST_DEG_TO_RAD, UNIT_TEST_PRECISION);
}
TEST(MathTool, CalculateAngleBetweenVectors) {
	Vector3d vec1 = Vector3d::Random();
	Vector3d vec2 = Vector3d::Random();

	double result_angle_deg = math_tool.CalculateAngleBetweenVectors(vec1, vec2); // return : (unit : deg)

	vec1.normalize();
	vec2.normalize();
	double expected_angle_rad = acos(vec1.dot(vec2));

	EXPECT_NEAR(expected_angle_rad, result_angle_deg * UNIT_TEST_DEG_TO_RAD, UNIT_TEST_PRECISION);
}

int main(int argc, char** argv) {
	::testing::InitGoogleTest(&argc, argv); // 初始化 Google Test 框架

	return RUN_ALL_TESTS(); // 運行所有測試
}