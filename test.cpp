#include <iostream>

#include "CoordinateTransformation.h"
#include "M5Manipulator.h"

using namespace std;

void ShowMatrix(const Matrix4d matrix) {
	printf("\
		%10.4lf , %10.4lf , %10.4lf , %10.4lf\n\
		%10.4lf , %10.4lf , %10.4lf , %10.4lf\n\
		%10.4lf , %10.4lf , %10.4lf , %10.4lf\n\
		%10.4lf , %10.4lf , %10.4lf , %10.4lf\n\n",
		matrix(0,0), matrix(0,1), matrix(0,2), matrix(0,3),
		matrix(1,0), matrix(1,1), matrix(1,2), matrix(1,3), 
		matrix(2,0), matrix(2,1), matrix(2,2), matrix(2,3), 
		matrix(3,0), matrix(3,1), matrix(3,2), matrix(3,3));
}

int main() {
	CoordinateTransformation coordinate_transformation;

	// set eye-in-hand parameter
	//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
	Matrix4d camera_robot_flange_to_camera = Matrix4d::Identity();
	camera_robot_flange_to_camera(0,0) = -0.717445;  camera_robot_flange_to_camera(0,1) = -0.691945;  camera_robot_flange_to_camera(0,2) =  0.0805244; camera_robot_flange_to_camera(0,3) = -148.387;
	camera_robot_flange_to_camera(1,0) =  0.689864;  camera_robot_flange_to_camera(1,1) = -0.721783;  camera_robot_flange_to_camera(1,2) = -0.0558274; camera_robot_flange_to_camera(1,3) =  224.923;
	camera_robot_flange_to_camera(2,0) =  0.0967507; camera_robot_flange_to_camera(2,1) =  0.0154978; camera_robot_flange_to_camera(2,2) =  0.995188;  camera_robot_flange_to_camera(2,3) =   64.9927;
	coordinate_transformation.SetCameraRobotFlangeToCameraTransMatrix(camera_robot_flange_to_camera);
	cout << "(camera_robot) flange_to_camera :" << endl;
	ShowMatrix(camera_robot_flange_to_camera);
	//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//

	// set (camera_robot) pose
	//--------------------------------------------------------------------------------------//
	Pose camera_robot_root_to_flange;
	camera_robot_root_to_flange.SetData(246.583, -173.780, 838.177, 124.923, -40.902, 70.430); // x,y,z,Rx,Ry,Rz

	cout << "(camera_robot) root_to_flange :" << endl;
	ShowMatrix(camera_robot_root_to_flange.GetData());
	//--------------------------------------------------------------------------------------//

	// set object information in camera frame
	//--------------------------------------------------------------------------------------------------------//
	Pose object_in_camera_frame;
	object_in_camera_frame.SetData(-90.058533, 136.486816, 902.947937, 0.697038, -0.389308, 0.347865, 0.491495); // x,y,z,qw,qx,qy,qz
	cout << "object_in_camera_frame :" << endl;
	ShowMatrix(object_in_camera_frame.GetData());
	//--------------------------------------------------------------------------------------------------------//

	// calculate object information in camera_robot root frame
	//-----------------------------------------------------------------------------------------------------------------------------------------//
	Pose camera_robot_root_to_object;
	coordinate_transformation.CalculateObjectInCameraRobotRoot(camera_robot_root_to_flange, object_in_camera_frame, camera_robot_root_to_object);
	cout << "(camera_robot) root_to_object :" << endl;
	ShowMatrix(camera_robot_root_to_object.GetData());
	//-----------------------------------------------------------------------------------------------------------------------------------------//


	// set gripper object information
	//----------------------------------------------------------------------------------------------------------//
	Matrix4d guide_robot_flange_to_gripper_object = Matrix4d::Identity();
	guide_robot_flange_to_gripper_object(2,3) = 100;
	coordinate_transformation.SetGuideRobotFlangeToGripperObjectTransMatrix(guide_robot_flange_to_gripper_object);
	//----------------------------------------------------------------------------------------------------------//

	/*****************************************************************************************************************************/

	//M5Manipulator m5;
	//Pose m5_robot_root_to_flange;

	//m5_robot_root_to_flange.SetData(1379.956, 289.447, 572.48);

	return 0;
}