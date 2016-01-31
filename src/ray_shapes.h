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

    Vector3 Eval(float t);

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
