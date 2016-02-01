#include "ray_shapes.h"

#include <utility>
#include <limits>

#pragma region Ray
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
#pragma endregion

#pragma region Helper
namespace
{
    using Interval = std::pair<real, real>;
    using Normals = std::pair<Vector3, Vector3>;

    bool IntersectRaySlab(
            const Ray& ray, 
            const Vector3& N, 
            real d0, 
            real d1, 
            Interval& interval,
            Normals& ns)
    {
        real divisor = N.dot(ray.GetDir());
        real N_dot_Q = N.dot(ray.GetPos());

        if (std::abs(divisor) > std::numeric_limits<real>::epsilon())
        {
            //ray goes through both planes

            real t0 = -(d0 + N_dot_Q) / divisor;
            real t1 = -(d1 + N_dot_Q) / divisor;

            if (t0 < t1)
            {
                interval = std::make_pair(t0,t1);
                ns = std::make_pair(-N,N);
            }
            else
            {
                interval = std::make_pair(t1,t0);
                ns = std::make_pair(N,-N);
            }
        }
        else
        {
            //ray parallel to planes
            real s0 = N_dot_Q + d0;
            real s1 = N_dot_Q + d1;

            if (std::signbit(s0) == std::signbit(s1))
            {
                //ray outside slab
                interval = std::make_pair(1,0);
                return false;
            }
            else
            {
                //ray inside slab
                interval = std::make_pair(0, std::numeric_limits<real>::infinity());
            }
        }

        return true;
    }
}
#pragma endregion

#pragma region Shapes
bool Sphere::Intersect(const Ray& ray, Intersection& intersection)
{
    Vector3 Q_mod = ray.GetPos() - m_c;

    real Q_dot_D = ray.GetDir().dot(Q_mod);
    real Q_dot_Q = Q_mod.dot(Q_mod);

    real discriminant = Q_dot_D * Q_dot_D - Q_dot_Q + m_r * m_r;
    if (discriminant <= 0)
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

AABB::AABB(Vector3 c, Vector3 diag, std::shared_ptr<Material> mat)
    : Shape(mat)
{
    Vector3 c2 = c + diag;

    m_min = c2.cwiseMin(c);
    m_max = c2.cwiseMax(c);
}

bool AABB::Intersect(const Ray& ray, Intersection& i)
{
    Interval interval = std::make_pair(0,std::numeric_limits<real>::infinity());
    Normals ns;

    for (int axis = 0; axis < 3; ++axis)
    {
        Vector3 N(axis==0, axis==1, axis==2);
        float d0 = -m_min(axis);
        float d1 = -m_max(axis);

        Interval slab;
        Normals slab_n;
        IntersectRaySlab(ray, N, d0, d1, slab, slab_n);

        if (interval.first < slab.first)
        {
            interval.first = slab.first;
            ns.first = slab_n.first;
        }
        if (interval.second > slab.second)
        {
            interval.second = slab.second;
            ns.second = slab_n.second;
        }

        if (interval.first >= interval.second)
            return false;
    }

    if (interval.first > 0)
    {
        i.t = interval.first;
        i.n = ns.first;
    }
    else if (interval.second > 0)
    {
        i.t = interval.second;
        i.n = ns.second;
    }
    else 
        return false;
    
    i.obj = this;
    i.p = ray.Eval(i.t);

    return true;
};
#pragma endregion
