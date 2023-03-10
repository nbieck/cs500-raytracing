#pragma once

#include "geom.h"
#include "material.h"

#include <memory>

const real PI = 3.14159f;

class Shape;

struct Intersection
{
    Intersection() : t(std::numeric_limits<real>::infinity()) {}
    real t;
    Vector3 p;
    Vector3 n;
    const Shape* obj;
};

class Ray
{
public:

    Ray(Vector3 p, Vector3 d);

    Vector3 Eval(float t) const;

    Vector3 GetPos() const;
    Vector3 GetDir() const;

private:

    Vector3 m_p;
    Vector3 m_d;
};

class Shape
{
public:
    
    virtual bool Intersect(const Ray& ray, Intersection& intersection) = 0;
    virtual BBox bounding_box() const = 0;
    
    virtual real surface() const {return 0;}
    virtual Intersection sample() const {return Intersection();}
    std::shared_ptr<Material> mat;

protected:
    Shape(std::shared_ptr<Material> material)
        : mat(material) {}
};

class Sphere : public Shape
{
public:
    Sphere(Vector3 c, real r, std::shared_ptr<Material> mat = nullptr)
        : Shape(mat), m_c(c), m_r(r) {}

    bool Intersect(const Ray& ray, Intersection& intersection) override;
    BBox bounding_box() const override;

    real surface() const override {return static_cast<real>(4) * PI * m_r * m_r;}
    Intersection sample() const override;
private:

    Vector3 m_c;
    real m_r;
};

class AABB : public Shape
{
public:

    AABB(Vector3 c, Vector3 diag, std::shared_ptr<Material> mat = nullptr);

    bool Intersect(const Ray& ray, Intersection& intersection) override;
    BBox bounding_box() const override;

private:
    Vector3 m_min;
    Vector3 m_max;
};

class Cylinder : public Shape
{
public:

    Cylinder(Vector3 B, Vector3 A, real R, std::shared_ptr<Material> m = nullptr);

    bool Intersect(const Ray& ray, Intersection& intersection) override;
    BBox bounding_box() const override;


private:

    Vector3 m_base;
    Quaternion m_orient;
    Vector3 m_axis;
    real m_r;
    real m_h;
};

class Triangle : public Shape
{
public:
    Triangle(std::shared_ptr<Material> mat) : Shape(mat) {}

    Triangle(Vector3 v0, Vector3 v1, Vector3 v2, std::shared_ptr<Material> mat = nullptr) 
        : Shape(mat), m_v0(v0), m_v1(v1), m_v2(v2) {}

    bool Intersect(const Ray& ray, Intersection& intersection) override;
    BBox bounding_box() const override;

    Vector3 m_v0, m_v1, m_v2;
    Vector3 m_n0, m_n1, m_n2;
};

using Interval = std::pair<real, real>;
using Normals = std::pair<Vector3, Vector3>;

bool IntersectRaySlab(
        const Ray& ray, 
        const Vector3& N, 
        real d0, 
        real d1, 
        Interval& interval,
        Normals& ns);
