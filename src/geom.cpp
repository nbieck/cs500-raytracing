#include <iostream>
#include "geom.h"

Quaternion angleAxis(const real& angle, const Vector3& axis)
{ return Quaternion(Eigen::AngleAxisf(angle, axis)); }

Matrix4 toMat4(const Quaternion& q)
{ return Eigen::Transform<real,3,Eigen::Affine>(q).matrix(); }

Matrix4 rotate(const real& angle, const Vector3& axis)
{  return Eigen::Transform<real,3,Eigen::Affine>(
                         Quaternion(Eigen::AngleAxisf(angle, axis))).matrix(); }
        
Matrix4 scale(const Vector3& v)
{  return Eigen::Transform<real,3,Eigen::Affine>(
                         Eigen::DiagonalMatrix<real,3>(v)).matrix(); }

Matrix4 translate(const Vector3& v)
{  return Eigen::Transform<real,3,Eigen::Affine>(Eigen::Translation<real,3>(v)).matrix(); }

// The standard glFrustum perspective projection matrix.
Matrix4 frustum(real const& left,    real const& right,
                 real const& bottom,  real const& top,
                 real const& nearVal, real const& farVal)
{
    Matrix4 R = Matrix4::Zero();
    R(0,0) = (2.0*nearVal)         / (right-left);
    R(1,1) = (2.0*nearVal)         / (top-bottom);
    R(0,2) = (right+left)          / (right-left);
    R(1,2) = (top+bottom)          / (top-bottom);
    R(2,2) = -(farVal+nearVal)     / (farVal-nearVal);
    R(3,2) = -1.0;
    R(2,3) = -(2.0*farVal*nearVal) / (farVal-nearVal);

    return R;
}

template <> void Print(const std::string& s, const real& f)
{ std::cout << s << ": " << f << std::endl; }

template <> void Print(const std::string& s, const double& f)
{ std::cout << s << ": " << f << std::endl; }

template <> void Print(const std::string& s, const Quaternion& q)
{
    Eigen::IOFormat Fmt(3, Eigen::DontAlignCols, ", ", "  |  ", "", "", ": [", "]\n");
    std::cout << s << ": " << q.w() << q.vec().format(Fmt); 
}

