#pragma once

#include <string>
#include <iostream>

#include <Eigen/StdVector> // For vectors, matrices (2d,3d,4d) and quaternions in f and d precision.
#include <unsupported/Eigen/BVH> // For KdBVH


///////////////////////////////////////////////////////////////////////
//typedefs
///////////////////////////////////////////////////////////////////////
using real = float;
using Color = Eigen::Array<real,3,1>;
using Bbox  = Eigen::AlignedBox<real,3>;
using Quaternion = Eigen::Quaternion<real>;
using Matrix4 = Eigen::Matrix<real,4,4>;
using Vector3 = Eigen::Matrix<real,3,1>;
using Vector2 = Eigen::Matrix<real,2,1>;

// Some convenience procedures: These are all provided by Eigen, but
// require two nested constructor calls to get the final result, with
// the intermediate being some form of a generic Transform<...> type.
Quaternion angleAxis(const real& angle, const Vector3& axis);
Matrix4 toMat4(const Quaternion& q);
Matrix4 rotate(const real& angle, const Vector3& axis);
Matrix4 scale(const Vector3& v);
Matrix4 translate(const Vector3& v);

// Eigen provided no perspective projection, so we build our own, paterned off glFrustum.
Matrix4 frustum(real const& left,    real const& right,
                 real const& bottom,  real const& top, 
                 real const& nearVal, real const& farVal);

// Debug Print() making use of Eigen's nice formatting class.
template <typename T> void Print(const std::string& s, const T& m)
{ 
    Eigen::IOFormat Fmt(3, Eigen::DontAlignCols, ", ", "  |  ", "", "", ": [", "]\n");
    std::cout << s << ": " << m.format(Fmt); 
}

template <> void Print(const std::string& s, const real& f);
template <> void Print(const std::string& s, const double& f);
template <> void Print(const std::string& s, const Quaternion& f);
