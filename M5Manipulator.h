#pragma once

#include "MathTool.h"

enum class ReachabilityResult {
    eNormal = 0,
    eOverHeightLimit = 1,
    eOverHorizontalLimit = 2
}; // end of enum


class M5Manipulator
{
    struct LinkParameters {
        double L; // unit : mm
        double D; // unit : mm
    }; // end of struct

    struct limit {
        double positve;
        double negative;
    }; // end of struct

public:
    M5Manipulator();
    ~M5Manipulator() {}

    ReachabilityResult AnalyzeReachability(const Pose m5_robot_root_to_flange);

private:
    MathTool math_tool_;

    LinkParameters link_parameters_[6] = {};

    limit vertical_limit_ = {};

    /*
     *   true : target height is under limit
     *  false : target height is over limit
     */
    bool CalculateVerticalCondition(const double position_z, double& vertical_angle) const;

}; // end of class