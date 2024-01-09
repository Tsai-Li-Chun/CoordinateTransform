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

void ShowMatrix(const Matrix4d matrix) {
	printf("\
		%10.4lf , %10.4lf , %10.4lf , %10.4lf\n\
		%10.4lf , %10.4lf , %10.4lf , %10.4lf\n\
		%10.4lf , %10.4lf , %10.4lf , %10.4lf\n\
		%10.4lf , %10.4lf , %10.4lf , %10.4lf\n\n",
		matrix(0, 0), matrix(0, 1), matrix(0, 2), matrix(0, 3),
		matrix(1, 0), matrix(1, 1), matrix(1, 2), matrix(1, 3),
		matrix(2, 0), matrix(2, 1), matrix(2, 2), matrix(2, 3),
		matrix(3, 0), matrix(3, 1), matrix(3, 2), matrix(3, 3));
}

// MathTool Unit Test
/*******************************************************************************************************/
MathTool math_tool;

TEST(UnitTest_MathTool, sind) {
	double rand_angle_deg = produce_random_value();
	double rand_angle_rad = rand_angle_deg * UNIT_TEST_DEG_TO_RAD;

	EXPECT_NEAR(sin(rand_angle_rad), math_tool.sind(rand_angle_deg), UNIT_TEST_PRECISION);
}
TEST(UnitTest_MathTool, cosd) {
	double rand_angle_deg = produce_random_value();
	double rand_angle_rad = rand_angle_deg * UNIT_TEST_DEG_TO_RAD;

	EXPECT_NEAR(cos(rand_angle_rad), math_tool.cosd(rand_angle_deg), UNIT_TEST_PRECISION);
}
TEST(UnitTest_MathTool, tand) {
	double rand_angle_deg = produce_random_value();
	double rand_angle_rad = rand_angle_deg * UNIT_TEST_DEG_TO_RAD;

	EXPECT_NEAR(tan(rand_angle_rad), math_tool.tand(rand_angle_deg), UNIT_TEST_PRECISION);
}
TEST(UnitTest_MathTool, asind) {
	double value = produce_random_value_in_range(1.0, -1.0);

	EXPECT_NEAR(asin(value), math_tool.asind(value) * UNIT_TEST_DEG_TO_RAD, UNIT_TEST_PRECISION);
}
TEST(UnitTest_MathTool, acosd) {
	double value = produce_random_value_in_range(1.0, -1.0);

	EXPECT_NEAR(acos(value), math_tool.acosd(value) * UNIT_TEST_DEG_TO_RAD, UNIT_TEST_PRECISION);
}
TEST(UnitTest_MathTool, atan2d) {
	double numerator   = produce_random_value(); // 分子
	double denominator = produce_random_value(); // 分母

	EXPECT_NEAR(atan2(numerator, denominator), math_tool.atan2d(numerator, denominator) * UNIT_TEST_DEG_TO_RAD, UNIT_TEST_PRECISION);
}
TEST(UnitTest_MathTool, CalculateAngleBetweenVectors) {
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

TEST(UnitTest_CoordinateTransformation, SetGet_CameraRobotFlangeToCameraTransMatrix) {
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
TEST(UnitTest_CoordinateTransformation, SetGet_GuideRobotFlangeToGripperObjectTransMatrix) {
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
TEST(UnitTest_CoordinateTransformation, CalibrateTwoRobotRoot) {
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
TEST(UnitTest_CoordinateTransformation, CalculateObjectInCameraRobotRoot) {
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
TEST(UnitTest_CoordinateTransformation, CalculateObjectInGuideRobotRoot) {
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


// CoordinateTransformation Integration Test
/*******************************************************************************************************/
TEST(IntegrationTest_CoordinateTransformation, CalibrateTwoRobotRoot) {
	// set eye-in-hand hand-eye calibration parameter
	//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
	Matrix4d camera_robot_flange_to_camera = Matrix4d::Identity();
	camera_robot_flange_to_camera(0, 0) = 0.025371;  camera_robot_flange_to_camera(0, 1) = -0.999671;    camera_robot_flange_to_camera(0, 2) =  0.00369405;	 camera_robot_flange_to_camera(0, 3) =  85.9135;
	camera_robot_flange_to_camera(1, 0) = 0.985612;  camera_robot_flange_to_camera(1, 1) =  0.0243961;   camera_robot_flange_to_camera(1, 2) = -0.167254;	 camera_robot_flange_to_camera(1, 3) = 271.064;
	camera_robot_flange_to_camera(2, 0) = 0.167108;  camera_robot_flange_to_camera(2, 1) =  0.00788429;  camera_robot_flange_to_camera(2, 2) =  0.985907;	 camera_robot_flange_to_camera(2, 3) =  81.1469;
	coordinate_transformation.SetCameraRobotFlangeToCameraTransMatrix(camera_robot_flange_to_camera);
	//std::cout << "(camera_robot) hand_eye_calibration_parameter :" << std::endl;
	//ShowMatrix(camera_robot_flange_to_camera);
	//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//


	// set two-robot root calibrated pose : camera_robot
	//------------------------------------------------------------------------------------------------//
	Pose camera_robot_root_to_flange_calibration;
	camera_robot_root_to_flange_calibration.SetData(228.235, 11.997, 1143.138, 179.997, -50.598, 3.009); // x,y,z,Rx,Ry,Rz
	//std::cout << "(camera_robot) root_to_flange :" << std::endl;
	//ShowMatrix(camera_robot_root_to_flange_calibration.GetData());
	//------------------------------------------------------------------------------------------------//

	// set calibrated (gripper) object information in camera frame
	//-----------------------------------------------------------------------------------------------------------------------------------------//
	Pose guide_robot_gripper_object_in_camera_frame_calibration;
	guide_robot_gripper_object_in_camera_frame_calibration.SetData(-225.805450, 29.109203, 970.318481, -0.069917, -0.292780, 0.836101, 0.458613); // x,y,z,qw,qx,qy,qz
	//std::cout << "calibrated_object_in_camera_frame :" << std::endl;
	//ShowMatrix(calibrated_object_in_camera_frame.GetData());
	//-----------------------------------------------------------------------------------------------------------------------------------------//


	// set gripper object information
	//----------------------------------------------------------------------------------------------------------//
	Matrix4d guide_robot_flange_to_gripper_object = Matrix4d::Identity();
	guide_robot_flange_to_gripper_object(2, 3) = 100.0;
	coordinate_transformation.SetGuideRobotFlangeToGripperObjectTransMatrix(guide_robot_flange_to_gripper_object);
	//----------------------------------------------------------------------------------------------------------//


	// set two-robot root calibrated pose : guid_robot
	//---------------------------------------------------------------------------------------------------//
	Pose guide_robot_root_to_flange_calib;
	guide_robot_root_to_flange_calib.SetData(703.321, 467.095, 350.559, -102.564, -37.113, -6.311);
	//std::cout << "(guide_robot) root_to_flange_calib :" << std::endl;
	//ShowMatrix(guide_robot_root_to_flange_calib.GetData());

	// calibrate two robot
	coordinate_transformation.CalibrateTwoRobotRoot(camera_robot_root_to_flange_calibration,
	                                                guide_robot_gripper_object_in_camera_frame_calibration,
	                                                guide_robot_root_to_flange_calib);
	//---------------------------------------------------------------------------------------------------//


	// get calibration info between two robot root
	//-----------------------------------------------------------------------------------------------------------------------//
	Matrix4d guide_robot_root_to_camera_robot_root = coordinate_transformation.GetGuideRobotRootToCameraRobotRootTransMatrix();
	//std::cout << "(guide_robot_) root to (camera_robot_) root :" << std::endl;
	//ShowMatrix(guide_robot_root_to_camera_robot_root);
	//-----------------------------------------------------------------------------------------------------------------------//


	//===================================================================================================================//
	// set (camera_robot) new pose
	Pose camera_robot_root_to_flange_new;
	camera_robot_root_to_flange_new.SetData(228.235, 11.997, 1143.138, 179.997, -50.598, 3.009); // x,y,z,Rx,Ry,Rz
	//std::cout << "new (camera_robot) root_to_flange :" << std::endl;
	//ShowMatrix(camera_robot_root_to_flange_new.GetData());

	// set new target object information in camera frame
	Pose targe_object_in_camera_frame_new;
	targe_object_in_camera_frame_new.SetData(-175.417755, 167.002106, 961.915039, 0.743212, -0.380590, 0.238759, 0.495763); // x,y,z,qw,qx,qy,qz
	//std::cout << "new target_object_in_camera_frame :" << std::endl;
	//ShowMatrix(targe_object_in_camera_frame_new.GetData());

	double x, y, z, Rx, Ry, Rz;

	Pose camera_robot_root_to_target_object_new;
	coordinate_transformation.CalculateObjectInCameraRobotRoot(camera_robot_root_to_flange_new,
	                                                           targe_object_in_camera_frame_new,
	                                                           camera_robot_root_to_target_object_new);
	//std::cout << "(camera_robot) root_to_new_target_object :" << std::endl;
	//ShowMatrix(camera_robot_root_to_target_object_new.GetData());

	camera_robot_root_to_target_object_new.GetData(x, y, z, Rx, Ry, Rz);
	//printf("(camera_robot) root_to_new_target_object pose : %lf , %lf , %lf , %lf , %lf , %lf\n\n\n", x, y, z, Rx, Ry, Rz);

	Pose guide_robot_root_to_target_object_new;
	coordinate_transformation.CalculateObjectInGuideRobotRoot(camera_robot_root_to_flange_new,
	                                                          targe_object_in_camera_frame_new,
	                                                          guide_robot_root_to_target_object_new);
	//std::cout << "(guide_robot) root_to_new_target_object :" << std::endl;
	//ShowMatrix(guide_robot_root_to_target_object_new.GetData());

	guide_robot_root_to_target_object_new.GetData(x, y, z, Rx, Ry, Rz);
	//printf("(guide_robot) root_to_new_target_object pose : %lf , %lf , %lf , %lf , %lf , %lf\n\n", x, y, z, Rx, Ry, Rz);
	//===================================================================================================================//
}
/*******************************************************************************************************/


// M5Manipulator Unit Test
/*******************************************************************************************************/
M5Manipulator m5_manipulator;

TEST(UnitTest_M5Manipulator, AnalyzeReachability_OverHeightLimit_Test) {
	ReachabilityResult reachability_result_actual = ReachabilityResult::eNormal;
	Limit vertical_height_limit = m5_manipulator.GetVerticalHeightLimit();
	Limit horizontal_limit = m5_manipulator.GetHorizontalLimit();
	Pose m5_robot_root_to_flange;

	double x = 0.0, y = 0.0, z = 0.0, Rx = 0.0, Ry = -90.0, Rz = 180.0;
	x = produce_random_value_in_range(horizontal_limit.positve, horizontal_limit.negative); // unit : mm
	y = produce_random_value_in_range(horizontal_limit.positve, horizontal_limit.negative); // unit : mm

	// exceed the vertical height limit test
	LinkParameters link_parameters[6];
	m5_manipulator.GetLinkParameters(link_parameters);
	double h = 0.0;
	double random_delta_height = produce_random_value(); // unit : mm
	h = random_delta_height >= 0.0 ? vertical_height_limit.positve : vertical_height_limit.negative;
	h += random_delta_height;
	z = h - link_parameters[5].D;
	Rx = produce_random_value_in_range(180.0, -180.0); // unit : deg

	m5_robot_root_to_flange.SetData(x, y, z, Rx, Ry, Rz);

	double max_distance_x_actual = 0.0;
	reachability_result_actual = m5_manipulator.AnalyzeReachability(m5_robot_root_to_flange, max_distance_x_actual);

	EXPECT_EQ(ReachabilityResult::eOverHeightLimit, reachability_result_actual);
}
TEST(UnitTest_M5Manipulator, AnalyzeReachability) {
	ReachabilityResult reachability_result_actual = ReachabilityResult::eNormal;
	Limit vertical_height_limit = m5_manipulator.GetVerticalHeightLimit(); // unit : mm
	Limit vertical_angle_limit = m5_manipulator.GetVerticalAngleLimit(); // unit : deg
	Limit horizontal_limit = m5_manipulator.GetHorizontalLimit();
	Pose m5_robot_root_to_flange;
	LinkParameters link_parameters[6];
	m5_manipulator.GetLinkParameters(link_parameters);

	double h = 0.0, x = 0.0, y = 0.0, z = 0.0, Rx = 0.0, Ry = -90.0, Rz = 180.0;
	double max_distance_x_actual = 0.0;
	x = produce_random_value_in_range(horizontal_limit.positve, horizontal_limit.negative); // unit : mm
	y = produce_random_value_in_range(horizontal_limit.positve, horizontal_limit.negative); // unit : mm
	h = produce_random_value_in_range(vertical_height_limit.positve, vertical_height_limit.negative); // unit : mm
	z = h - link_parameters[5].D; // z = h - D6
	Rx = produce_random_value_in_range(180.0, -180.0); // unit : deg

	m5_robot_root_to_flange.SetData(x, y, z, Rx, Ry, Rz);
	reachability_result_actual = m5_manipulator.AnalyzeReachability(m5_robot_root_to_flange, max_distance_x_actual);

	// test the condition for exceeding vertical height limit
	//-------------------------------------------------------------------------------------------------------------------------------//
	if (((h - vertical_height_limit.negative) < -UNIT_TEST_PRECISION) || ((h - vertical_height_limit.positve) > UNIT_TEST_PRECISION)) {
		EXPECT_EQ(ReachabilityResult::eOverHeightLimit, reachability_result_actual);
		return;
	}
	//-------------------------------------------------------------------------------------------------------------------------------//

	double tmp = h - link_parameters[0].D; // h - D1
	double vertical_angle_rad = asin(tmp / link_parameters[3].L);

	// test the condition for exceeding vertical angle limit
	//--------------------------------------------------------------------------------//
	double vertical_angle_deg = vertical_angle_rad * UNIT_TEST_RAD_TO_DEG;
	if (((vertical_angle_deg - vertical_angle_limit.negative) < -UNIT_TEST_PRECISION) ||
	    ((vertical_angle_deg - vertical_angle_limit.positve) > UNIT_TEST_PRECISION)) {
		EXPECT_EQ(ReachabilityResult::eOverHeightLimit, reachability_result_actual);
		return;
	}
	//--------------------------------------------------------------------------------//

	Vector3d position = m5_robot_root_to_flange.GetPosition();
	Matrix3d rotation = m5_robot_root_to_flange.GetRotation();

	Vector3d uz = rotation.block<3,1>(0,2);
	Vector3d uy = Vector3d::Zero();
	uy(1) = y / fabs(y);

	double alpha_rad = acos(uz.dot(uy));
	double cos_alpha = cos(alpha_rad);

	double B = link_parameters[2].L + link_parameters[3].L * cos(vertical_angle_rad) + link_parameters[4].L + link_parameters[5].L;
	double E = link_parameters[0].L + link_parameters[1].L;
	double target_distance = sqrt(x * x + y * y);

	ReachabilityResult reachability_result_expected;

	if (target_distance > B + E) {
		EXPECT_EQ(ReachabilityResult::eOverHorizontalLimit, reachability_result_actual);
		return;
	} else {
		double C = fabs(y);
		double A = sqrt(B * B + C * C - 2 * B * C * cos_alpha);

		double cos_beta = (C - B * cos_alpha) / A;
		double sin_beta = sqrt(1 - cos_beta * cos_beta);

		double discriminant = (A * sin_beta) * (A * sin_beta) - (A * A - E * E);

		if (discriminant < 0) {
			EXPECT_EQ(ReachabilityResult::eOverHorizontalLimit, reachability_result_actual);
		} else {
			double tmp = sqrt(discriminant);
			double D1 = A * sin_beta + tmp;
			double D2 = A * sin_beta - tmp;
			double D = D1 >= D2 ? D1 : D2;
			double max_distance_x_expected = D;

			reachability_result_expected = (fabs(x) - D) > UNIT_TEST_PRECISION ? ReachabilityResult::eOverHorizontalLimit : ReachabilityResult::eNormal;

			EXPECT_NEAR(max_distance_x_expected, max_distance_x_actual, UNIT_TEST_PRECISION);
		}
	}
}
void AnalyzeReachabilityStressTest(double& out_x, double& out_y, double& out_z, double& out_Rx, double& out_Ry, double& out_Rz) {
	ReachabilityResult reachability_result_actual = ReachabilityResult::eNormal;
	Limit vertical_height_limit = m5_manipulator.GetVerticalHeightLimit(); // unit : mm
	Limit vertical_angle_limit = m5_manipulator.GetVerticalAngleLimit(); // unit : deg
	Limit horizontal_limit = m5_manipulator.GetHorizontalLimit();
	Pose m5_robot_root_to_flange;
	LinkParameters link_parameters[6];
	m5_manipulator.GetLinkParameters(link_parameters);

	double h = 0.0, x = 0.0, y = 0.0, z = 0.0, Rx = 0.0, Ry = -90.0, Rz = 180.0;
	double max_distance_x_actual = 0.0;
	x = produce_random_value_in_range(horizontal_limit.positve, horizontal_limit.negative); // unit : mm
	y = produce_random_value_in_range(horizontal_limit.positve, horizontal_limit.negative); // unit : mm
	h = produce_random_value_in_range(vertical_height_limit.positve, vertical_height_limit.negative); // unit : mm
	z = h - link_parameters[5].D; // z = h - D6
	Rx = produce_random_value_in_range(180.0, -180.0); // unit : deg

	m5_robot_root_to_flange.SetData(x, y, z, Rx, Ry, Rz);
	reachability_result_actual = m5_manipulator.AnalyzeReachability(m5_robot_root_to_flange, max_distance_x_actual);

	// test vertical limit condition
	//-----------------------------------------------------------------------------------------------------------------//
	if (((h - vertical_height_limit.negative) < -UNIT_TEST_PRECISION) || ((h - vertical_height_limit.positve) > UNIT_TEST_PRECISION)) {
		EXPECT_EQ(ReachabilityResult::eOverHeightLimit, reachability_result_actual);

		out_x = x;  out_y = y;  out_z = z;
		out_Rx = Rx;  out_Ry = Ry;  out_Rz = Rz;
		return;
	}
	//-----------------------------------------------------------------------------------------------------------------//

	double tmp = h - link_parameters[0].D; // h - D1
	double vertical_angle_rad = asin(tmp / link_parameters[3].L);

	// test the condition for exceeding vertical angle limit
	//--------------------------------------------------------------------------------//
	double vertical_angle_deg = vertical_angle_rad * UNIT_TEST_RAD_TO_DEG;
	if (((vertical_angle_deg - vertical_angle_limit.negative) < -UNIT_TEST_PRECISION) ||
	    ((vertical_angle_deg - vertical_angle_limit.positve) > UNIT_TEST_PRECISION)) {
		EXPECT_EQ(ReachabilityResult::eOverHeightLimit, reachability_result_actual);
		return;
	}
	//--------------------------------------------------------------------------------//

	Vector3d position = m5_robot_root_to_flange.GetPosition();
	Matrix3d rotation = m5_robot_root_to_flange.GetRotation();

	Vector3d uz = rotation.block<3,1>(0,2);
	Vector3d uy = Vector3d::Zero();
	uy(1) = y / fabs(y);

	double alpha_rad = acos(uz.dot(uy));
	double cos_alpha = cos(alpha_rad);

	double B = link_parameters[2].L + link_parameters[3].L * cos(vertical_angle_rad) + link_parameters[4].L + link_parameters[5].L;
	double E = link_parameters[0].L + link_parameters[1].L;
	double target_distance = sqrt(x * x + y * y);

	ReachabilityResult reachability_result_expected;

	if (target_distance > B + E) {
		EXPECT_EQ(ReachabilityResult::eOverHorizontalLimit, reachability_result_actual);

		out_x = x;  out_y = y;  out_z = z;
		out_Rx = Rx;  out_Ry = Ry;  out_Rz = Rz;
		return;
	} else {
		double C = fabs(y);
		double A = sqrt(B * B + C * C - 2 * B * C * cos_alpha);

		double cos_beta = (C - B * cos_alpha) / A;
		double sin_beta = sqrt(1 - cos_beta * cos_beta);

		double discriminant = (A * sin_beta) * (A * sin_beta) - (A * A - E * E);

		if (discriminant < 0) {
			EXPECT_EQ(ReachabilityResult::eOverHorizontalLimit, reachability_result_actual);

			out_x = x;  out_y = y;  out_z = z;
			out_Rx = Rx;  out_Ry = Ry;  out_Rz = Rz;
			return;
		} else {
			double tmp = sqrt(discriminant);
			double D1 = A * sin_beta + tmp;
			double D2 = A * sin_beta - tmp;
			double D = D1 >= D2 ? D1 : D2;
			double max_distance_x_expected = D;

			reachability_result_expected = (fabs(x) - D) > UNIT_TEST_PRECISION ? ReachabilityResult::eOverHorizontalLimit : ReachabilityResult::eNormal;

			EXPECT_NEAR(max_distance_x_expected, max_distance_x_actual, UNIT_TEST_PRECISION);

			out_x = x;  out_y = y;  out_z = z;
			out_Rx = Rx;  out_Ry = Ry;  out_Rz = Rz;
			return;
		}
	}
}
TEST(UnitTest_M5Manipulator, AnalyzeReachability_StressTesting) {
	int count = 0;
	int count_limit = 1000000; // max testing times
	double x, y, z, Rx, Ry, Rz;

	while (true) {
		x = 0.0, y = 0.0, z = 0.0, Rx = 0.0, Ry = 0.0, Rz = 0.0; // reset

		AnalyzeReachabilityStressTest(x, y, z, Rx, Ry, Rz);

		if (::testing::Test::HasFailure() || count > count_limit) {
			if (count > count_limit) {
				printf("=== Tested %d times. ===\n", count);
			} else {
				printf("Tested fail @ %d_th time. The pose is : x = %lf , y = %lf , z = %lf , Rx = %lf , Ry = %lf , Rz = %lf\n", count, x, y, z, Rx, Ry, Rz);
			}

			break;
		}

		count++;
	}
}
/*******************************************************************************************************/


// M5Manipulator Integration Test
/*******************************************************************************************************/
TEST(IntegrationTest_M5Manipulator, AnalyzeReachability) {
	ReachabilityResult reachability_result_actual = ReachabilityResult::eNormal;
	Pose m5_robot_root_to_flange;

	double x = 100 /*1075.44463612023*/, y = 1063.24515890219, z = 491.868215745095, Rx = 30.0, Ry = -90.0, Rz = 180.0; // vertical angle : 35 deg
	//double x = 100 /*1075.44463612023*/, y = 1063.24515890219, z = -150.528215745095, Rx = 30.0, Ry = -90.0, Rz = 180.0; // vertical angle : -35 deg

	double max_distance_x_actual = 0.0;
	m5_robot_root_to_flange.SetData(x, y, z, Rx, Ry, Rz);
	reachability_result_actual = m5_manipulator.AnalyzeReachability(m5_robot_root_to_flange, max_distance_x_actual);

	double max_distance_x_expected = 1075.44463612023; // theoretical maximum distance

	EXPECT_NEAR(max_distance_x_expected, max_distance_x_actual, UNIT_TEST_PRECISION);
}
/*******************************************************************************************************/

int main(int argc, char** argv) {
	::testing::InitGoogleTest(&argc, argv); // 初始化 Google Test 框架

	return RUN_ALL_TESTS(); // 運行所有測試
}