#include <gtest/gtest.h>

#include "../ray_shapes.h"

TEST(ray_sphere_intersections, ray_behind_sphere_pointing_backwards)
{
    Ray ray(Vector3(-2,0,0), Vector3(-1,0,0));
    Sphere sphere(Vector3(0,0,0), 1);
    Intersection i;

    ASSERT_FALSE(sphere.Intersect(ray, i));
}

TEST(ray_sphere_intersections, ray_behind_sphere_pointing_forwards)
{
    Ray ray(Vector3(-2, 0, 0), Vector3(1, 0, 0));
    Sphere sphere(Vector3(0,0,0), 1);
    Intersection i;

    ASSERT_TRUE(sphere.Intersect(ray, i));
    ASSERT_FLOAT_EQ(1, i.t);
    ASSERT_EQ(&sphere, i.obj);
}

TEST(ray_sphere_intersections, ray_at_sphere_center)
{
    Ray ray(Vector3(0,0,0), Vector3(1,0,0));
    Sphere sphere(Vector3(0,0,0), 1);
    Intersection i;

    ASSERT_TRUE(sphere.Intersect(ray, i));
    ASSERT_FLOAT_EQ(1, i.t);
}
