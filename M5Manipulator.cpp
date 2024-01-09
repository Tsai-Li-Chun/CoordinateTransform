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

	vertical_angle_limit_.positve  =  39.0; // unit : deg
	vertical_angle_limit_.negative = -39.0; // unit : deg

	// height_limit = D1 + L4 * sin(vertical_angle_limit)
	vertical_heigth_limit_.positve  = link_parameters_[0].D + link_parameters_[3].L * math_tool_.sind(vertical_angle_limit_.positve);
	vertical_heigth_limit_.negative = link_parameters_[0].D + link_parameters_[3].L * math_tool_.sind(vertical_angle_limit_.negative);

	horizontal_length_limit_.positve = link_parameters_[0].L +
	                                   link_parameters_[1].L +
	                                   link_parameters_[2].L +
	                                   link_parameters_[3].L +
	                                   link_parameters_[4].L;
	horizontal_length_limit_.negative = - horizontal_length_limit_.positve;
}


ReachabilityResult M5Manipulator::AnalyzeReachability(const Pose m5_robot_root_to_flange, double& max_distance_x) const {
	double vertical_angle = 0.0; // theta
	Vector3d position = m5_robot_root_to_flange.GetPosition();
	Matrix3d rotation = m5_robot_root_to_flange.GetRotation();

	bool is_over_height_limit = CalculateVerticalCondition(position(2), vertical_angle);
	if (is_over_height_limit == true) {
		max_distance_x = 0.0;
		return ReachabilityResult::eOverHeightLimit;
	}

	Vector3d uz = rotation.block<3,1>(0,2);
	Vector3d uy = Vector3d::Zero();
	uy(1) = position(1) / fabs(position(1));

	double alpha = math_tool_.CalculateAngleBetweenVectors(uz, uy); // unit : deg
	double cos_alpha = math_tool_.cosd(alpha);

	// B : horizontal projection of the vertical component = L3 + L4*cos(theta) + L5 + L6
	double B = link_parameters_[2].L + link_parameters_[3].L * math_tool_.cosd(vertical_angle) + link_parameters_[4].L + link_parameters_[5].L;
	double E = link_parameters_[0].L + link_parameters_[1].L;
	double target_distance = sqrt(position(0)*position(0) + position(1)*position(1));

	if (target_distance > B + E) {
		max_distance_x = 0.0;
		return ReachabilityResult::eOverHorizontalLimit;
	}

	double C = fabs(position(1)); // y component
	double A = sqrt(B * B + C * C - 2 * B * C * cos_alpha);

	double cos_beta = (C - B * cos_alpha) / A;
	double sin_beta = sqrt(1 - cos_beta * cos_beta);

	double discriminant = (A * sin_beta) * (A * sin_beta) - (A * A - E * E);

	if (discriminant < 0) {
		max_distance_x = 0.0;
		return ReachabilityResult::eOverHorizontalLimit;
	} else {
		double tmp = sqrt((A * sin_beta) * (A * sin_beta) - (A * A - E * E));
		double D1 = A * sin_beta + tmp;
		double D2 = A * sin_beta - tmp;
		double D = D1 >= D2 ? D1 : D2;

		max_distance_x = D;

		return (fabs(position(0)) - max_distance_x) > ERROR_PRECISION ? ReachabilityResult::eOverHorizontalLimit : ReachabilityResult::eNormal;
	}
}

void M5Manipulator::GetLinkParameters(LinkParameters link_parameters[6]) const {
	for (int i = 0; i < LINK_NUM; ++i) {
		link_parameters[i] = link_parameters_[i];
	}
}

bool M5Manipulator::CalculateVerticalCondition(const double position_z, double& vertical_angle) const {
	double h = position_z + link_parameters_[5].D; // h = z + D6

	if (CHECK_LIMIT(h, vertical_heigth_limit_.positve, vertical_heigth_limit_.negative) == true) {
		vertical_angle = 0.0;
		return true; // true : target height is exceeding limit
	}

	double tmp = h - link_parameters_[0].D; // h - D1
	vertical_angle = math_tool_.asind(tmp / link_parameters_[3].L);

	//  true : target height is exceeding the limit
	// false : target height is within the limit
	return CHECK_LIMIT(vertical_angle, vertical_angle_limit_.positve, vertical_angle_limit_.negative);
}
