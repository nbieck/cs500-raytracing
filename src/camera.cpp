#include "camera.h"

#include "raytrace.h"

Camera::Camera(const Vector3& eye, const Quaternion& q, real ratio)
    : m_eye(eye), m_q(q), m_ratio(ratio) 
{
    X = m_q._transformVector(Vector3::UnitX());
    Y = m_q._transformVector(Vector3::UnitY());
    Z = m_q._transformVector(Vector3::UnitZ());
}

void Camera::SetDims(int width, int height)
{
    m_width = width;
    m_height = height;
    ratio_x = (m_ratio * width) / height;
}

Ray Camera::MakeRay(int x, int y)
{
    real d_x = 2 * (x + static_cast<real>(1) / 2) / m_width - 1;
    real d_y = 2 * (y + static_cast<real>(1) / 2) / m_height - 1;

    return Ray(m_eye, d_x * ratio_x * X +  d_y * m_ratio * Y - Z);
}

Ray Camera::MakeRayAA(int x, int y)
{
    real d_x = 2 * (x + myrandom(RNGen)) / m_width - 1;
    real d_y = 2 * (y + myrandom(RNGen)) / m_height - 1;

    return Ray(m_eye, d_x * ratio_x * X + d_y * m_ratio * Y - Z);
}
