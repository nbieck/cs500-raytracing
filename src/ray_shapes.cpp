#include "ray_shapes.h"

Ray::Ray(Vector3 p, Vector3 d)
    : m_p(p), m_d(d.normalized())
{}

Vector3 Ray::Eval(float t) const
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
    Vector3 Q_mod = ray.GetPos() - m_c;

    real Q_dot_D = ray.GetDir().dot(Q_mod);
    real Q_dot_Q = Q_mod.dot(Q_mod);

    real discriminant = Q_dot_D * Q_dot_D - Q_dot_Q + m_r * m_r;
    if (discriminant < 0)
        return false;

    real root = std::sqrt(discriminant);
    real t_0 = -Q_dot_D - root;
    real t_1 = -Q_dot_D + root;

    if (t_1 <= 0)
        return false;

    if (t_0 > 0)
        intersection.t = t_0;
    else 
        intersection.t = t_1;

    intersection.p = ray.Eval(intersection.t);
    intersection.obj = this;
    intersection.n = (intersection.p - m_c).normalized();

    return true;
}

bool AABB::Intersect(const Ray& ray, Intersection& i)
{
    return false;
};
#pragma endregion
