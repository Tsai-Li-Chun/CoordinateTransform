#include "M5Manipulator.h"

M5Manipulator::M5Manipulator() {
	link_parameters_[0].L = 641.15;
	link_parameters_[0].D = 193.26;

	link_parameters_[1].L = 125.0;
	link_parameters_[2].L = 133.877;
	link_parameters_[3].L = 559.992;
	link_parameters_[4].L = 115.114;

	link_parameters_[5].L = 91.77;
	link_parameters_[5].D = 22.59;

	vertical_limit_.positve = 528.365;
	vertical_limit_.negative = -191.547;
}


ReachabilityResult M5Manipulator::AnalyzeReachability(const Pose m5_robot_root_to_flange) {
	double vertical_angle = 0.0; // theta
	Vector3d position = m5_robot_root_to_flange.GetPosition();
	Matrix3d rotation = m5_robot_root_to_flange.GetRotation();

	bool is_over_height_limit = CalculateVerticalCondition(position(2), vertical_angle);

	if (is_over_height_limit == true) {
		return ReachabilityResult::eOverHeightLimit;
	}

	Vector3d flange_z_direction = rotation.block<3,1>(0,2);
	Vector3d flange_position_y_component = Vector3d::Zero();
	flange_position_y_component(1) = position(1) / fabs(position(1));

	double alpha = math_tool_.CalculateAngleBetweenVectors(flange_z_direction, flange_position_y_component);

	// B : horizontal projection of the vertical component = L3 + L4*cos(theta) + L5 + L6
	double B = link_parameters_[2].L + link_parameters_[3].L * math_tool_.cosd(vertical_angle) + link_parameters_[4].L + link_parameters_[5].L;
	double C = fabs(position(1)); // y component
	double A = sqrt(B * B + C * C - 2 * B * C * math_tool_.cosd(alpha));

	double cos_beta = (C - B * math_tool_.cosd(alpha)) / A;
	double sin_beta = sqrt(1 - cos_beta * cos_beta);

	double E = link_parameters_[0].L + link_parameters_[1].L;
	double tmp = sqrt((A * sin_beta) * (A * sin_beta) - (A * A - E * E));
	double D1 = A * sin_beta + tmp;
	double D2 = A * sin_beta - tmp;
	double D = D1 >= D2 ? D1 : D2;

	return fabs(position(0)) <= D ? ReachabilityResult::eNormal : ReachabilityResult::eOverHorizontalLimit;
}

bool M5Manipulator::CalculateVerticalCondition(const double position_z, double& vertical_angle) const {
	if (position_z < vertical_limit_.negative || position_z > vertical_limit_.positve) {
		return true;
	}
	vertical_angle = math_tool_.asind((position_z + link_parameters_[5].D - link_parameters_[0].D) / link_parameters_[3].L);
	return false;
}