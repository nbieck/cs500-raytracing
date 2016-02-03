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

BBox Sphere::bounding_box()
{
    return BBox(m_c + Vector3(m_r,m_r,m_r), m_c - Vector3(m_r,m_r,m_r));
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

BBox AABB::bounding_box()
{
    return BBox(m_min, m_max);
}

Cylinder::Cylinder(Vector3 B, Vector3 A, real R, std::shared_ptr<Material> m)
    : Shape(m)
{
    m_axis = A;
    m_base = B;
    m_h = A.norm();
    m_r = R;
    m_orient = Quaternion::FromTwoVectors(A.normalized(), Vector3::UnitZ());
}

bool Cylinder::Intersect(const Ray& ray, Intersection& i)
{
    Interval interval = std::make_pair(0,std::numeric_limits<real>::infinity());
    Normals ns;

    Ray conv_ray(m_orient._transformVector(ray.GetPos() - m_base), 
            m_orient._transformVector(ray.GetDir()));

    Interval slab;
    Normals slab_n;
    if (!IntersectRaySlab(conv_ray, Vector3(0,0,1), 0, -m_h, slab, slab_n))
        return false;

    if (slab.first > interval.first)
    {
        interval.first = slab.first;
        ns.first = slab_n.first;
    }
    if (slab.second < interval.second)
    {
        interval.second = slab.second;
        ns.second = slab_n.second;
    }

    Vector3 D = conv_ray.GetDir();
    Vector3 Q = conv_ray.GetPos();
    real a = D.x() * D.x() + D.y() * D.y();
    real b = 2 * (D.x() * Q.x() + D.y() * Q.y());
    real c = Q.x() * Q.x() + Q.y() * Q.y() - m_r * m_r;
    real discriminant = b * b - 4 * a * c;
    if (discriminant <= 0)
        return false;

    real root = std::sqrt(discriminant);
    real t0 = (-b - root)/(2 * a);
    real t1 = (-b + root)/(2 * a);

    if (t0 > interval.first)
    {
        interval.first = t0;
        Vector3 M = conv_ray.Eval(t0);
        ns.first = Vector3(M.x(), M.y(), 0).normalized();
    }
    if (t1 < interval.second)
    {
        interval.second = t1;
        Vector3 M = conv_ray.Eval(t1);
        ns.second = Vector3(M.x(), M.y(), 0).normalized();
    }

    if (interval.second <= interval.first)
        return false;

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

    i.n = m_orient.conjugate()._transformVector(i.n);
    i.p = ray.Eval(i.t);
    i.obj = this;

    return true;
}

BBox Cylinder::bounding_box()
{
    BBox result(m_base + Vector3(m_r,m_r,m_r), m_base - Vector3(m_r,m_r,m_r));
    Vector3 other_end = m_base + m_axis;
    result.extend(m_axis + Vector3(m_r,m_r,m_r));
    result.extend(m_axis - Vector3(m_r,m_r,m_r));

    return result;
}

bool Triangle::Intersect(const Ray& ray, Intersection& i)
{
    Vector3 e1 = m_v1 - m_v0;
    Vector3 e2 = m_v2 - m_v0;
    Vector3 p = ray.GetDir().cross(e2);
    real d = p.dot(e1);
    if (std::abs(d) < std::numeric_limits<real>::epsilon())
        return false;

    Vector3 S = ray.GetPos() - m_v0;
    real u = p.dot(S) / d;
    if (u <= 0 || u >= 1)
        return false;

    Vector3 q = S.cross(e1);
    real v = (ray.GetDir().dot(q)) / d;
    if (v <= 0 || (u + v) >= 1)
        return false;

    real t = e2.dot(q) / d;
    if (t < 0)
        return false;

    i.t = t;
    i.obj = this;
    i.p = ray.Eval(t);
    i.n = (1 - u - v) * m_n0 + u * m_n1 + v * m_n2;
    i.n.normalize();

    return true;
}

BBox Triangle::bounding_box()
{
    BBox result(m_v0);
    result.extend(m_v1);
    result.extend(m_v2);

    return result;
}
#pragma endregion
