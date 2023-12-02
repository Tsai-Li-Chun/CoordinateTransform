#pragma once

#include "Define.h"

class MathTool
{
public:
    MathTool();
    ~MathTool();
	double CalculateAngleBetweenVectors(const Vector3d vec1, const Vector3d vec2) const; // return : (unit : deg)

    inline double sind(const double value) const { return sin(value * DEG_TO_RAD); }
    inline double cosd(const double value) const { return cos(value * DEG_TO_RAD); }
    inline double tand(const double value) const { return tan(value * DEG_TO_RAD); }

    inline double asind(const double value) const { return asin(value) * RAD_TO_DEG; }
    inline double acosd(const double value) const { return acos(value) * RAD_TO_DEG; }
    inline double atan2d(const double value1, const double value2) const { return atan2(value1, value2) * RAD_TO_DEG; }

private:
    

}; // end of class