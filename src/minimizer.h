#pragma once

#include "geom.h"
#include "ray_shapes.h"

class Minimizer
{
public:
    using Scalar = real;
    Ray ray;
    Intersection min_i;

    Minimizer(const Ray& r) : ray(r) {}

    real minimumOnObject(const std::shared_ptr<Shape>& obj);
    real minimumOnVolume(const BBox& box);
};

inline BBox bounding_box(const std::shared_ptr<Shape>& obj)
{
    return obj->bounding_box();
}

