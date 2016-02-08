#include "minimizer.h"

real Minimizer::minimumOnObject(const std::shared_ptr<Shape>& obj)
{
    Intersection i;
    obj->Intersect(ray, i);
    if (i.t < min_i.t)
        min_i = i;

    return i.t;
}

real Minimizer::minimumOnVolume(const BBox& box)
{
    Interval interval = std::make_pair(0,std::numeric_limits<real>::infinity());

    for (int axis = 0; axis < 3; ++axis)
    {
        Vector3 N(axis==0, axis==1, axis==2);
        float d0 = -box.corner(box.BottomLeftFloor)(axis);
        float d1 = -box.corner(box.TopRightCeil)(axis);

        Interval slab;
        Normals slab_n;
        IntersectRaySlab(ray, N, d0, d1, slab, slab_n);

        if (interval.first < slab.first)
        {
            interval.first = slab.first;
        }
        if (interval.second > slab.second)
        {
            interval.second = slab.second;
        }

        if (interval.first > interval.second)
            return std::numeric_limits<real>::infinity();
    }

    if (interval.first > 0)
    {
        return interval.first;
    }
    else if (interval.second > 0)
    {
        return 0;
    }

    return std::numeric_limits<real>::infinity();
}
