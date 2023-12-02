#include "MathTool.h"

MathTool::MathTool() {

}

MathTool::~MathTool() {

}

double MathTool::CalculateAngleBetweenVectors(const Vector3d vec1, const Vector3d vec2) const {
    return acosd(vec1.dot(vec2) / (vec1.norm() * vec2.norm()));
}