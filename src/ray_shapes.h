#pragma once

#include "geom.h"
#include "material.h"

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
    
    Material* mat;
};

class Sphere : public Shape
{
public:
    Sphere(Vector3 c, real r)
        : m_c(c), m_r(r) {}

    bool Intersect(const Ray& ray, Intersection& intersection) override;

private:

    Vector3 m_c;
    real m_r;
};

class AABB : public Shape
{
public:

    AABB(Vector3 c, Vector3 diag)
        : m_c(c), m_diag(diag) {}

    bool Intersect(const Ray& ray, Intersection& intersection) override;

private:
    Vector3 m_c;
    Vector3 m_diag;
};

class Cylinder : public Shape
{
};

class Triangle : public Shape
{
};
