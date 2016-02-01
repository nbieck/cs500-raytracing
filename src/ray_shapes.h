#pragma once

#include "geom.h"
#include "material.h"

#include <memory>

class Shape;

struct Intersection
{
    real t;
    Vector3 p;
    Vector3 n;
    Shape* obj;
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

private:

    Vector3 m_c;
    real m_r;
};

class AABB : public Shape
{
public:

    AABB(Vector3 c, Vector3 diag, std::shared_ptr<Material> mat = nullptr);

    bool Intersect(const Ray& ray, Intersection& intersection) override;

private:
    Vector3 m_min;
    Vector3 m_max;
};

class Cylinder : public Shape
{
public:

    Cylinder(Vector3 B, Vector3 A, real R, std::shared_ptr<Material> m = nullptr)
        :Shape(m) {}

    bool Intersect(const Ray& ray, Intersection& intersection) override;

private:
};

class Triangle : public Shape
{
};
