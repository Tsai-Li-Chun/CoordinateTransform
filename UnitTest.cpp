#include "pch.h"

#include <iostream>
#include <cstdlib> /* �üƬ������ */
#include <ctime>   /* �ɶ�������� */

#include "MathTool.h"

#define UNIT_TEST_PI (atan(1) * 4)
#define UNIT_TEST_RAD_TO_DEG (1.0 / UNIT_TEST_PI * 180.0)
#define UNIT_TEST_DEG_TO_RAD (1.0 / UNIT_TEST_RAD_TO_DEG)

#define UNIT_TEST_PRECISION (1e-7)

MathTool math_tool;

// ���ͫ��w�d�� [min , max] ���B�I�ƶü�
double inline produce_random_value_in_range(const double max, const double min) {
	srand(time(NULL)); // �H�ɶ�����¦���üƺؤl
	// rand() : [0 , RAND_MAX]
	// rand() / RAND_MAX : ���ͽd�� [0 , 1] ���B�I�ƶü�
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
	double numerator   = produce_random_value(); // ���l
	double denominator = produce_random_value(); // ����

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
	::testing::InitGoogleTest(&argc, argv); // ��l�� Google Test �ج[

	return RUN_ALL_TESTS(); // �B��Ҧ�����
}