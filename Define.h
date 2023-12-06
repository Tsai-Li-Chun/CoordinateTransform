#pragma once

#if defined(_WIN32)
    #include "Eigen/Dense"
#elif defined (__linux__)
    #include "eigen3/Eigen/Dense"
#endif

#define PI (3.14159265359)
#define RAD_TO_DEG (1.0 / PI * 180.0)
#define DEG_TO_RAD (1.0 / RAD_TO_DEG)

using namespace Eigen;

struct Orientation {
public:
    Orientation() { // constructor
        Reset();
    }
    void Reset() {
        rx_ = 0.0, rx_ = 0.0, rz_ = 0.0;
        quat_.setIdentity();
        rotation_.setIdentity();
    }

    void SetData(const Matrix3d rotation) {
        rotation_ = rotation;

        double rx, ry, rz;
        if (rotation_to_euler_zyx(rotation_, rx, ry, rz) == true) {
            rx_ = rx;
            ry_ = ry;
            rz_ = rz;
        }

        quat_ = Quaterniond(rotation);
    }
    void SetData(const double rx, const double ry, const double rz) { // input (unit : deg)
         rx_ = rx * DEG_TO_RAD; 
         ry_ = ry * DEG_TO_RAD; 
         rz_ = rz * DEG_TO_RAD;

        rotation_ = AngleAxisd(rz_, Vector3d::UnitZ()) *
                    AngleAxisd(ry_, Vector3d::UnitY()) *
                    AngleAxisd(rx_, Vector3d::UnitX());

        quat_ = Quaterniond(rotation_);
    }
    void SetData(const double qw, const double qx, const double qy, const double qz) {
        quat_.w() = qw;
        quat_.x() = qx;
        quat_.y() = qy;
        quat_.z() = qz;

        rotation_ = quat_.toRotationMatrix();

        double rx, ry, rz;
        if (rotation_to_euler_zyx(rotation_, rx, ry, rz) == true) {
            rx_ = rx;
            ry_ = ry;
            rz_ = rz;
        }
    }

    Matrix3d GetData() const {
        return rotation_;
    }
    void GetData(Matrix3d& rotation) const {
        rotation = rotation_;
    }
    void GetData(double& rx, double& ry, double& rz) const { // output (unit : deg)
        rx = rx_ * RAD_TO_DEG;
        ry = ry_ * RAD_TO_DEG;
        rz = rz_ * RAD_TO_DEG;
    }
    void GetData(double& qw, double& qx, double& qy, double& qz) const {
        qw = quat_.w();
        qx = quat_.x();
        qy = quat_.y();
        qz = quat_.z();
    }

private:
    double rx_, ry_, rz_; // euler : z-y-x (unit : rad)
    Quaterniond quat_;
    Matrix3d rotation_;

    bool rotation_to_euler_zyx(const Matrix3d rotation, double& rx, double& ry, double& rz) {
        double RAD2mDEG = RAD_TO_DEG * 1000.0;
        double mDEG2RAD = 1.0 / RAD2mDEG;
        double Rx, Ry, Rz; // unit : mDeg
        // SCARA 5軸沒有 B 的自由度旋轉，除非使用者設定座標系有y方向旋轉，不然一律進入 else 的判斷
        if ((fabs(rotation(0, 0)) - 1.7e-5) < 0 && (fabs(rotation(1, 0)) - 1.7e-5) < 0) // code from C67
        {
            // 選 A 為0的解比較適合
            if (rotation(2, 0) >= 0)
            {
                Ry = -90000; // -pi/2
                Rx = 0;
                if (rotation(1, 2) == 0 && rotation(0, 2) == 0)  return false;
                Rz = (atan2(-rotation(1, 2), -rotation(0, 2)) * RAD2mDEG);
            }
            else if (rotation(2, 0) < 0)
            {
                Ry = 90000;  //  pi/2
                Rx = 0;
                if (rotation(1, 2) == 0 && rotation(0, 2) == 0)  return false;
                Rz = (atan2(rotation(1, 2), rotation(0, 2)) * RAD2mDEG);
            }
        }
        else
        {
            if (-rotation(2, 0) == 0 && (1 - rotation(2, 0) * rotation(2, 0)) == 0)  return false;
            if ((1 - rotation(2, 0) * rotation(2, 0)) < 0)
            {
                int a;//sqrt zero
                a++;
            }
            Ry = (atan2(-rotation(2, 0), sqrt(1 - rotation(2, 0) * rotation(2, 0))) * RAD2mDEG); //pitch angle
            if (cos(Ry * mDEG2RAD) > 0.0f) {
                if (rotation(2, 1) == 0 && rotation(2, 2) == 0)  return false;
                Rx = (atan2(rotation(2, 1), rotation(2, 2)) * RAD2mDEG);//roll  angle
                if (rotation(1, 0) == 0 && rotation(0, 0) == 0)  return false;
                Rz = (atan2(rotation(1, 0), rotation(0, 0)) * RAD2mDEG);//yaw   angle
            }
            else if (cos(Ry * mDEG2RAD) < 0.0f) {
                if (-rotation(2, 1) == 0 && -rotation(2, 2) == 0)  return false;
                Rx = (atan2(-rotation(2, 1), -rotation(2, 2)) * RAD2mDEG);//roll  angle
                if (-rotation(1, 0) == 0 && -rotation(0, 0) == 0)  return false;
                Rz = (atan2(-rotation(1, 0), -rotation(0, 0)) * RAD2mDEG);//yaw   angle
            }
        }
        // 三維空間 A,C自由度在正負180度，視為同一方位...
        if (fabs(Rx + 180000) < 0.001) {
            Rx = 180000;
        }
        if (fabs(Rz + 180000) < 0.001) {
            Rz = 180000;
        }

        rx = Rx * mDEG2RAD; 
        ry = Ry * mDEG2RAD;
        rz = Rz * mDEG2RAD;
        return true;
    }
}; // end of struct



struct Pose {
public:
    Pose() { // constructor
        Reset();
    }
    void Reset() {
        position_.setZero();
        orientation_.Reset();
        transformation_.setIdentity();
    }

    void SetData(const Matrix4d transformation) {
        transformation_ = transformation;

        orientation_.SetData((Matrix3d)transformation.block<3,3>(0,0));

        position_ = transformation.block<3,1>(0,3);
    }
    void SetData(const double x, const double y, const double z, const double rx, const double ry, const double rz) {
        position_ << x, y, z;

        orientation_.SetData(rx, ry, rz);

        transformation_.block<3,1>(0,3) = position_;
        transformation_.block<3,3>(0,0) = orientation_.GetData();
    }
    void SetData(const double x, const double y, const double z, const double qw, const double qx, const double qy, const double qz) {
        position_ << x, y, z;

        orientation_.SetData(qw, qx, qy, qz);

        transformation_.block<3,1>(0,3) = position_;
        transformation_.block<3,3>(0,0) = orientation_.GetData();
    }

    Matrix4d GetData() const { // HT : Homogeneous Transformation
        return transformation_;
    }
    void GetData(Matrix4d& transformation) const {
        transformation = transformation_;
    }
    void GetData(double& x, double& y, double& z, double& rx, double& ry, double& rz) const {
        x = position_(0);
        y = position_(1);
        z = position_(2);

        orientation_.GetData(rx, ry, rz);
    }

    Vector3d GetPosition() const {
        return position_;
    }
    Matrix3d GetRotation() const {
        return orientation_.GetData();
    }
private:
    Vector3d position_;
    Orientation orientation_;
    Matrix4d transformation_;
}; // end of struct
