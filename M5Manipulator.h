#pragma once

#include "MathTool.h"

#define LINK_NUM (6)

enum class ReachabilityResult {
    eNormal = 0,
    eOverHeightLimit = 1,
    eOverHorizontalLimit = 2
}; // end of enum

struct Limit {
    double positve;
    double negative;
}; // end of struct

struct LinkParameters {
    double L; // unit : mm
    double D; // unit : mm
}; // end of struct

class M5Manipulator
{
public:
    M5Manipulator();
    ~M5Manipulator() {}

    ReachabilityResult AnalyzeReachability(const Pose m5_robot_root_to_flange, double& max_distance_x) const;
    Limit GetVerticalHeightLimit() const { return vertical_heigth_limit_; }
    Limit GetVerticalAngleLimit() const { return vertical_angle_limit_; }
    Limit GetHorizontalLimit() const { return horizontal_length_limit_; }
    void GetLinkParameters(LinkParameters link_parameters[6]) const;

private:
    MathTool math_tool_;

    LinkParameters link_parameters_[LINK_NUM] = {};

    Limit vertical_angle_limit_ = {};
    Limit vertical_heigth_limit_ = {};
    Limit horizontal_length_limit_ = {};

    /*
     *   true : target height is exceeding the limit
     *  false : target height is within the limit
     */
    bool CalculateVerticalCondition(const double position_z, double& vertical_angle) const;

}; // end of class