#include "pch.h"

#include <iostream>
#include <cstdlib> /* 亂數相關函數 */
#include <ctime>   /* 時間相關函數 */

#include "CoordinateTransformation.h"

#define UNIT_TEST_PI (atan(1) * 4)
#define UNIT_TEST_RAD_TO_DEG (1.0 / UNIT_TEST_PI * 180.0)
#define UNIT_TEST_DEG_TO_RAD (1.0 / UNIT_TEST_RAD_TO_DEG)

#define UNIT_TEST_PRECISION (1e-7)

int random_seed = time(NULL); // 以時間為基礎的亂數種子
int count = 0;

// 產生指定範圍 [min , max] 的 '浮點數' 亂數
double inline produce_random_value_in_range(const double max, const double min) {
	srand(random_seed + count);
	count++;
	// rand() : [0 , RAND_MAX]
	// rand() / RAND_MAX : 產生範圍 [0 , 1] 的浮點數亂數
	return (max - min) * (double) rand() / RAND_MAX + min;
}
double inline  produce_random_value() {
	return produce_random_value_in_range(0.5 * RAND_MAX, -0.5 * RAND_MAX);
}
// 產生指定範圍 [min , max] 的 '整數' 亂數
int inline produce_int_random_value_in_range(const int max, const int min) {
	srand(random_seed + count);
	count++;
	return rand() % (max - min + 1) + min;
}

// MathTool Unit Test
/*******************************************************************************************************/
MathTool math_tool;

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
	Vector3d vec1 = Vector3d::Zero();
	Vector3d vec2 = Vector3d::Zero();
	vec1.setRandom();
	vec2.setRandom();

	double actual_angle_deg = math_tool.CalculateAngleBetweenVectors(vec1, vec2); // return : (unit : deg)

	vec1.normalize();
	vec2.normalize();
	double expected_angle_rad = acos(vec1.dot(vec2));

	EXPECT_NEAR(expected_angle_rad, actual_angle_deg * UNIT_TEST_DEG_TO_RAD, UNIT_TEST_PRECISION);
}
/*******************************************************************************************************/


// CoordinateTransformation Unit Test
/*******************************************************************************************************/
CoordinateTransformation coordinate_transformation;

TEST(CoordinateTransformation, SetGet_CameraRobotFlangeToCameraTransMatrix) {
	double x, y, z, Rx, Ry, Rz;
	x = produce_random_value();
	y = produce_random_value();
	z = produce_random_value();
	Rx = produce_random_value();
	Ry = produce_random_value();
	Rz = produce_random_value();
	Vector3d position_xyz = Vector3d::Zero();
	Vector3d rotation_RxRyRz = Vector3d::Zero();
	position_xyz << x, y, z;
	rotation_RxRyRz << Rx, Ry, Rz;

	Matrix4d expected_trans_matrix = Matrix4d::Identity();
	expected_trans_matrix.block<3,1>(0,3) = position_xyz;
	Matrix3d rotation_matrix = AngleAxisd(rotation_RxRyRz(2) * UNIT_TEST_DEG_TO_RAD, Vector3d::UnitZ()).toRotationMatrix() *
							   AngleAxisd(rotation_RxRyRz(1) * UNIT_TEST_DEG_TO_RAD, Vector3d::UnitY()).toRotationMatrix() *
							   AngleAxisd(rotation_RxRyRz(0) * UNIT_TEST_DEG_TO_RAD, Vector3d::UnitX()).toRotationMatrix();
	expected_trans_matrix.block<3,3>(0,0) = rotation_matrix;

	coordinate_transformation.SetCameraRobotFlangeToCameraTransMatrix(expected_trans_matrix);

	Matrix4d actual_trans_matrix = coordinate_transformation.GetCameraRobotFlangeToCameraTransMatrix();

	bool is_approximate = actual_trans_matrix.isApprox(expected_trans_matrix, UNIT_TEST_PRECISION);

	EXPECT_EQ(true, is_approximate);
}
TEST(CoordinateTransformation, SetGet_GuideRobotFlangeToGripperObjectTransMatrix) {
	double x, y, z, Rx, Ry, Rz;
	x = produce_random_value();
	y = produce_random_value();
	z = produce_random_value();
	Rx = produce_random_value();
	Ry = produce_random_value();
	Rz = produce_random_value();
	Vector3d position_xyz = Vector3d::Zero();
	Vector3d rotation_RxRyRz = Vector3d::Zero();
	position_xyz << x, y, z;
	rotation_RxRyRz << Rx, Ry, Rz;

	Matrix4d expected_trans_matrix = Matrix4d::Identity();
	expected_trans_matrix.block<3,1>(0,3) = position_xyz;
	Matrix3d rotation_matrix = AngleAxisd(rotation_RxRyRz(2) * UNIT_TEST_DEG_TO_RAD, Vector3d::UnitZ()).toRotationMatrix() *
							   AngleAxisd(rotation_RxRyRz(1) * UNIT_TEST_DEG_TO_RAD, Vector3d::UnitY()).toRotationMatrix() *
							   AngleAxisd(rotation_RxRyRz(0) * UNIT_TEST_DEG_TO_RAD, Vector3d::UnitX()).toRotationMatrix();
	expected_trans_matrix.block<3,3>(0,0) = rotation_matrix;

	coordinate_transformation.SetGuideRobotFlangeToGripperObjectTransMatrix(expected_trans_matrix);

	Matrix4d actual_trans_matrix = coordinate_transformation.GetGuideRobotFlangeToGripperObjectTransMatrix();

	bool is_approximate = actual_trans_matrix.isApprox(expected_trans_matrix, UNIT_TEST_PRECISION);

	EXPECT_EQ(true, is_approximate);
}
TEST(CoordinateTransformation, CalibrateTwoRobotRoot) {
	double x, y, z, Rx, Ry, Rz;

	Pose camera_robot_root_to_flange;
	x = produce_random_value();
	y = produce_random_value();
	z = produce_random_value();
	Rx = produce_random_value();
	Ry = produce_random_value();
	Rz = produce_random_value();
	camera_robot_root_to_flange.SetData(x, y, z, Rx, Ry, Rz);

	Pose camera_to_gripper_object;
	x = produce_random_value();
	y = produce_random_value();
	z = produce_random_value();
	Rx = produce_random_value();
	Ry = produce_random_value();
	Rz = produce_random_value();
	camera_to_gripper_object.SetData(x, y, z, Rx, Ry, Rz);

	Pose guide_robot_root_to_flange;
	x = produce_random_value();
	y = produce_random_value();
	z = produce_random_value();
	Rx = produce_random_value();
	Ry = produce_random_value();
	Rz = produce_random_value();
	guide_robot_root_to_flange.SetData(x, y, z, Rx, Ry, Rz);

	coordinate_transformation.CalibrateTwoRobotRoot(camera_robot_root_to_flange, camera_to_gripper_object, guide_robot_root_to_flange);
	Matrix4d guide_robot_root_to_camera_robot_root_actual = coordinate_transformation.GetGuideRobotRootToCameraRobotRootTransMatrix();


	Matrix4d camera_robot_flange_to_camera = coordinate_transformation.GetCameraRobotFlangeToCameraTransMatrix();
	Matrix4d guide_robot_flange_to_gripper_object = coordinate_transformation.GetGuideRobotFlangeToGripperObjectTransMatrix();

	Matrix4d camera_robot_root_to_gripper_object = camera_robot_root_to_flange.GetData() * camera_robot_flange_to_camera * camera_to_gripper_object.GetData();

	Matrix4d guide_robot_root_to_gripper_object = guide_robot_root_to_flange.GetData() * guide_robot_flange_to_gripper_object;

	Matrix4d guide_robot_root_to_camera_robot_root_expected = guide_robot_root_to_gripper_object * camera_robot_root_to_gripper_object.inverse();

	bool is_approximate = guide_robot_root_to_camera_robot_root_actual.isApprox(guide_robot_root_to_camera_robot_root_expected, UNIT_TEST_PRECISION);

	EXPECT_EQ(true, is_approximate);
}
TEST(CoordinateTransformation, CalculateObjectInCameraRobotRoot) {
	double x, y, z, Rx, Ry, Rz;

	Pose camera_robot_root_to_flange;
	x = produce_random_value();
	y = produce_random_value();
	z = produce_random_value();
	Rx = produce_random_value();
	Ry = produce_random_value();
	Rz = produce_random_value();
	camera_robot_root_to_flange.SetData(x, y, z, Rx, Ry, Rz);

	Pose camera_to_object;
	x = produce_random_value();
	y = produce_random_value();
	z = produce_random_value();
	Rx = produce_random_value();
	Ry = produce_random_value();
	Rz = produce_random_value();
	camera_to_object.SetData(x, y, z, Rx, Ry, Rz);

	Pose camera_robot_root_to_object_actual;
	coordinate_transformation.CalculateObjectInCameraRobotRoot(camera_robot_root_to_flange, camera_to_object, camera_robot_root_to_object_actual);

	Matrix4d camera_robot_root_to_object_expected = Matrix4d::Identity();
	Matrix4d camera_robot_flange_to_camera = coordinate_transformation.GetCameraRobotFlangeToCameraTransMatrix();
	camera_robot_root_to_object_expected = camera_robot_root_to_flange.GetData() * camera_robot_flange_to_camera * camera_to_object.GetData();

	bool is_approximate = camera_robot_root_to_object_actual.GetData().isApprox(camera_robot_root_to_object_expected, UNIT_TEST_PRECISION);

	EXPECT_EQ(true, is_approximate);
}
TEST(CoordinateTransformation, CalculateObjectInGuideRobotRoot) {
	double x, y, z, Rx, Ry, Rz;

	Pose camera_robot_root_to_flange;
	x = produce_random_value();
	y = produce_random_value();
	z = produce_random_value();
	Rx = produce_random_value();
	Ry = produce_random_value();
	Rz = produce_random_value();
	camera_robot_root_to_flange.SetData(x, y, z, Rx, Ry, Rz);

	Pose camera_to_object;
	x = produce_random_value();
	y = produce_random_value();
	z = produce_random_value();
	Rx = produce_random_value();
	Ry = produce_random_value();
	Rz = produce_random_value();
	camera_to_object.SetData(x, y, z, Rx, Ry, Rz);

	Pose guide_robot_root_to_object_actual;
	coordinate_transformation.CalculateObjectInGuideRobotRoot(camera_robot_root_to_flange, camera_to_object, guide_robot_root_to_object_actual);


	Matrix4d camera_robot_root_to_object = Matrix4d::Identity();
	Matrix4d camera_robot_flange_to_camera = coordinate_transformation.GetCameraRobotFlangeToCameraTransMatrix();
	camera_robot_root_to_object = camera_robot_root_to_flange.GetData() * camera_robot_flange_to_camera * camera_to_object.GetData();

	Matrix4d guide_robot_root_to_object_expected = Matrix4d::Identity();
	Matrix4d guide_robot_root_to_camera_robot_root = coordinate_transformation.GetGuideRobotRootToCameraRobotRootTransMatrix();
	guide_robot_root_to_object_expected = guide_robot_root_to_camera_robot_root * camera_robot_root_to_object;

	bool is_approximate = guide_robot_root_to_object_actual.GetData().isApprox(guide_robot_root_to_object_expected, UNIT_TEST_PRECISION);

	EXPECT_EQ(true, is_approximate);
}
/*******************************************************************************************************/

int main(int argc, char** argv) {
	::testing::InitGoogleTest(&argc, argv); // 初始化 Google Test 框架

	return RUN_ALL_TESTS(); // 運行所有測試
}