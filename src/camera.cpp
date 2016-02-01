#include "camera.h"

Camera::Camera(const Vector3& eye, const Quaternion& q, real ratio)
    : m_eye(eye), m_q(q), m_ratio(ratio) 
{
    X = m_q._transformVector(Vector3::UnitX());
    Y = m_q._transformVector(Vector3::UnitY());
    Z = m_q._transformVector(Vector3::UnitZ());
}

Ray Camera::MakeRay(int width, int height, int x, int y)
{
    real ratio_x = (m_ratio * width) / height;

    real d_x = 2 * (x + static_cast<real>(1) / 2) / width - 1;
    real d_y = 2 * (y + static_cast<real>(1) / 2) / height - 1;

    return Ray(m_eye, d_x * ratio_x * X +  d_y * m_ratio * Y - Z);
}
