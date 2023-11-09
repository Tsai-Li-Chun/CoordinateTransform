#include "CoordinateTransformation.h"

CoordinateTransformation::CoordinateTransformation() {
    camera_robot_flange_to_camera_.setIdentity();
    guide_robot_flange_to_gripper_object_.setIdentity();
    guide_robot_root_to_camera_robot_root_.setIdentity();
}

void CoordinateTransformation::SetCameraRobotFlangeToCameraTransMatrix(const Matrix4d flange_to_camera) {
    camera_robot_flange_to_camera_ = flange_to_camera;
}

void CoordinateTransformation::SetGuideRobotFlangeToGripperObjectTransMatrix(const Matrix4d flange_to_gripper_object) {
    guide_robot_flange_to_gripper_object_ = flange_to_gripper_object;
}

void CoordinateTransformation::CalibrateTwoRobotRoot(const Pose camera_robot_root_to_flange, const Pose camera_to_gripper_object, const Pose guide_robot_root_to_flange) {
    // camera_robot :
    // {root}_T_{gripper_obj}  =  {root}_T_{flange}  *  {flange}_T_{camera}  *  {camera}_T_{gripper_obj}
    //----------------------------------------------------------------------------------------------------------------------------------------------//
    Matrix4d camera_robot_root_to_gripper_object = Matrix4d::Identity();
    camera_robot_root_to_gripper_object = camera_robot_root_to_flange.GetData() * camera_robot_flange_to_camera_ * camera_to_gripper_object.GetData();
    //----------------------------------------------------------------------------------------------------------------------------------------------//


    // guide_robot:
    // {root}_T_{gripper_obj}  =  {root}_T_{flange}  *  {flange}_T_{gripper_obj}
    //--------------------------------------------------------------------------------------------------------------//
    Matrix4d guide_robot_root_to_gripper_object = Matrix4d::Identity();
    guide_robot_root_to_gripper_object = guide_robot_root_to_flange.GetData() * guide_robot_flange_to_gripper_object_;
    //--------------------------------------------------------------------------------------------------------------//


    // {guide_robot_root}_T_{camera_robot_root}  =  {guide_robot_root}_T_{gripper_obj}  *  {gripper_obj}_T_{camera_robot_root}
    //                                           =  {guide_robot_root}_T_{gripper_obj}  *  inverse( {camera_robot_root}_T_{gripper_obj} )
    //------------------------------------------------------------------------------------------------------------------------//
    guide_robot_root_to_camera_robot_root_ = guide_robot_root_to_gripper_object * camera_robot_root_to_gripper_object.inverse();
    //------------------------------------------------------------------------------------------------------------------------//
}


void CoordinateTransformation::CalculateObjectInCameraRobotRoot(const Pose camera_robot_root_to_flange, const Pose camera_to_object, Pose& camera_robot_root_to_object) const {
    // camera_robot :
    // {root}_T_{obj}  =  {root}_T_{flange}  *  {flange}_T_{camera}  *  {camera}_T_{object}
    Matrix4d root_to_obj = Matrix4d::Identity();
    root_to_obj = camera_robot_root_to_flange.GetData() * camera_robot_flange_to_camera_ * camera_to_object.GetData();

    camera_robot_root_to_object.SetData(root_to_obj);
}

void CoordinateTransformation::CalculateObjectInGuideRobotRoot(const Pose camera_robot_root_to_flange, const Pose camera_to_object, Pose& guide_robot_root_to_object) const {
    // calculate : {camera_root}_T_{object}
    Pose camera_robot_root_to_object;
    CalculateObjectInCameraRobotRoot(camera_robot_root_to_flange, camera_to_object, camera_robot_root_to_object);

    // {guide_root}_T_{obj}  =  {guide_root}_T_{camera_root}  *  {camera_root}_T_{object}
    Matrix4d guide_robot_root_to_obj = Matrix4d::Identity();
    guide_robot_root_to_obj = guide_robot_root_to_camera_robot_root_ * camera_robot_root_to_object.GetData();
}
