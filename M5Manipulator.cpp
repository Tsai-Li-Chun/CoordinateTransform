#include "M5Manipulator.h"

M5Manipulator::M5Manipulator() {
	link_parameters_[0].L = 640.0;
	link_parameters_[0].D = 197.0;
	
	link_parameters_[1].L = 125.0;
	link_parameters_[2].L = 134.0;
	link_parameters_[3].L = 550.0;
	link_parameters_[4].L = 115.0;

	link_parameters_[5].L = 107.0;
	link_parameters_[5].D =  17.5;

	vertical_limit_.positve  =  533.0;
	vertical_limit_.negative = -174.0;
}


ReachabilityResult M5Manipulator::AnalyzeReachability(const Pose m5_robot_root_to_flange) {
	double vertical_angle = 0.0; // theta
	Vector3d position = m5_robot_root_to_flange.GetPosition();
	Matrix3d rotation = m5_robot_root_to_flange.GetRotation();

	bool is_over_height_limit = CalculateVerticalCondition(position(2), vertical_angle);

	if (is_over_height_limit == true) {
		return ReachabilityResult::eOverHeightLimit;
	}

	double alpha = math_tool_.acosd(rotation(2,1));

	// B : horizontal projection of the vertical component = L3 + L4*cos(theta) + L5
	double B = link_parameters_[2].L + link_parameters_[3].L*math_tool_.cosd(vertical_angle) + link_parameters_[4].L;
	double C = position(1); // y component
	double A = sqrt(B*B + C*C - 2*B*C*math_tool_.cosd(alpha));

	double cos_beta = ( C - B*math_tool_.cosd(alpha) ) / A;
	double sin_beta = sqrt(1 - cos_beta*cos_beta);

	double E = link_parameters_[0].L + link_parameters_[1].L;
	double tmp = sqrt( (A*sin_beta)*(A*sin_beta) - (A*A - E*E) );
	double D1 = A*sin_beta + tmp;
	double D2 = A*sin_beta - tmp;
	double D = D1 >= D2 ? D1 : D2;

	return position(0) <= D ? ReachabilityResult::eNormal : ReachabilityResult::eOverHorizontalLimit;
}

bool M5Manipulator::CalculateVerticalCondition(const double position_z, double& vertical_angle) const {
	if (position_z < vertical_limit_.negative || position_z > vertical_limit_.positve) {
		return false;
	}
	
	vertical_angle = math_tool_.asind( (position_z + link_parameters_[5].D - link_parameters_[0].D) / link_parameters_[3].L);
	return true;
}