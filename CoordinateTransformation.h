#pragma once

#include "M5Manipulator.h"

class CoordinateTransformation
{
public:
    CoordinateTransformation(); // constructor
    ~CoordinateTransformation() {}

    void SetCameraRobotFlangeToCameraTransMatrix(const Matrix4d flange_to_camera); // input : eye-in-hand calibration result
    void SetGuideRobotFlangeToGripperObjectTransMatrix(const Matrix4d flange_to_gripper_object); // input : get from CAD
    void CalibrateTwoRobotRoot(const Pose camera_robot_root_to_flange, const Pose camera_to_gripper_object, const Pose guide_robot_root_to_flange);

    void CalculateObjectInCameraRobotRoot(const Pose camera_robot_root_to_flange, const Pose camera_to_object, Pose& camera_robot_root_to_object) const;
    void CalculateObjectInGuideRobotRoot(const Pose camera_robot_root_to_flange, const Pose camera_to_object, Pose& guide_robot_root_to_object) const;

private:
    Matrix4d camera_robot_flange_to_camera_;
    Matrix4d guide_robot_flange_to_gripper_object_;
    Matrix4d guide_robot_root_to_camera_robot_root_;
}; // end of class