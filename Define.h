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
        euler_.setZero();
        quat_.setIdentity();
        rotation_.setIdentity();
    }

    void SetData(const Matrix3d rotation) {
        rotation_ = rotation;

        euler_ = rotation.eulerAngles(2, 1, 0); // 2 : Z  ,  1 : Y  ,  0 : X

        quat_ = Quaterniond(rotation);
    }
    void SetData(const double rx, const double ry, const double rz) { // input (unit : deg)
        euler_ << rx * DEG_TO_RAD, ry * DEG_TO_RAD, rz * DEG_TO_RAD;

        rotation_ = AngleAxisd(euler_(2), Vector3d::UnitZ()) *
                    AngleAxisd(euler_(1), Vector3d::UnitY()) *
                    AngleAxisd(euler_(0), Vector3d::UnitX());

        quat_ = Quaterniond(rotation_);
    }
    void SetData(const double qw, const double qx, const double qy, const double qz) {
        quat_.w() = qw;
        quat_.x() = qx;
        quat_.y() = qy;
        quat_.z() = qz;

        rotation_ = quat_.toRotationMatrix();

        euler_ = rotation_.eulerAngles(2, 1, 0);
    }

    Matrix3d GetData() const {
        return rotation_;
    }
    void GetData(Matrix3d& rotation) const {
        rotation = rotation_;
    }
    void GetData(double& rx, double& ry, double& rz) const { // output (unit : deg)
        rx = euler_(0) * RAD_TO_DEG;
        ry = euler_(1) * RAD_TO_DEG;
        rz = euler_(2) * RAD_TO_DEG;
    }
    void GetData(double& qw, double& qx, double& qy, double& qz) const {
        qw = quat_.w();
        qx = quat_.x();
        qy = quat_.y();
        qz = quat_.z();
    }

private:
    Vector3d euler_; // euler z-y-x (unit : rad)
    Quaterniond quat_;
    Matrix3d rotation_;
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
