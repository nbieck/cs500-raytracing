#include <gtest/gtest.h>

#include "../ray_shapes.h"

TEST(ray_aabb_intersections, ray_behind_aabb_pointing_back)
{
    Ray ray(Vector3(-2,0,0), Vector3(-1,0,0));
    AABB aabb(Vector3(-1,-1,-1),Vector3(2,2,2));
    Intersection i;

    ASSERT_FALSE(aabb.Intersect(ray, i));
}

TEST(ray_aabb_intersection, ray_behind_aabb_pointing_forwards)
{
    Ray ray(Vector3(-2,0.5,0), Vector3(1,0,0));
    AABB aabb(Vector3(-1,-1,-1), Vector3(2,2,2));
    Intersection i;

    ASSERT_TRUE(aabb.Intersect(ray, i));
    ASSERT_FLOAT_EQ(1,i.t);
    ASSERT_EQ(&aabb, i.obj);
}

TEST(ray_aabb_intersection, ray_inside_aabb)
{
    Ray ray(Vector3(0.5,-0.5,0), Vector3(1,0,0));
    AABB aabb(Vector3(-1,-1,-1), Vector3(2,2,2));
    Intersection i;

    ASSERT_TRUE(aabb.Intersect(ray, i));
    ASSERT_FLOAT_EQ(0.5,i.t);
}
