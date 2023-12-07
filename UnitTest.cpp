#include "pch.h"

#include <iostream>
#include <cstdlib> /* �üƬ������ */
#include <ctime>   /* �ɶ�������� */

#include "CoordinateTransformation.h"

#define UNIT_TEST_PI (atan(1) * 4)
#define UNIT_TEST_RAD_TO_DEG (1.0 / UNIT_TEST_PI * 180.0)
#define UNIT_TEST_DEG_TO_RAD (1.0 / UNIT_TEST_RAD_TO_DEG)

#define UNIT_TEST_PRECISION (1e-7)

int random_seed = time(NULL); // �H�ɶ�����¦���üƺؤl
int count = 0;

// ���ͫ��w�d�� [min , max] �� '�B�I��' �ü�
double inline produce_random_value_in_range(const double max, const double min) {
	srand(random_seed + count);
	count++;
	// rand() : [0 , RAND_MAX]
	// rand() / RAND_MAX : ���ͽd�� [0 , 1] ���B�I�ƶü�
	return (max - min) * (double) rand() / RAND_MAX + min;
}
double inline  produce_random_value() {
	return produce_random_value_in_range(0.5 * RAND_MAX, -0.5 * RAND_MAX);
}
// ���ͫ��w�d�� [min , max] �� '���' �ü�
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
	double numerator   = produce_random_value(); // ���l
	double denominator = produce_random_value(); // ����

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



int main(int argc, char** argv) {
	::testing::InitGoogleTest(&argc, argv); // ��l�� Google Test �ج[

	return RUN_ALL_TESTS(); // �B��Ҧ�����
}