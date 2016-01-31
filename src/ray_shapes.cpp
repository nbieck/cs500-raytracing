#include "ray_shapes.h"

Ray::Ray(Vector3 p, Vector3 d)
    : m_p(p), m_d(d.normalized())
{}

Vector3 Ray::Eval(float t)
{
    return m_p + t * m_d;
}

Vector3 Ray::GetPos() const
{
    return m_p;
}

Vector3 Ray::GetDir() const
{
    return m_d;
}

#pragma region Shapes
bool Sphere::Intersect(const Ray& ray, Intersection& intersection)
{
    return false;
}
#pragma endregion
