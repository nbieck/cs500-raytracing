#include <gtest/gtest.h>

#include "../ray_shapes.h"

TEST(ray_triangle, ray_misses_triangle)
{
    Ray ray(Vector3(0,-1,4), Vector3(0,1,1));
    Triangle triangle(Vector3(0,0,0), Vector3(-3,2,0), Vector3(0,4,1));
    Intersection i;

    ASSERT_FALSE(triangle.Intersect(ray, i));
}

TEST(ray_triangle, ray_touching_triangle)
{
    Ray ray(Vector3(0,-1,0), Vector3(0,1,0));
    Triangle triangle(Vector3(-1,0,0), Vector3(1,0,0), Vector3(0,1,1));
    Intersection i;

    ASSERT_FALSE(triangle.Intersect(ray, i));
}

TEST(ray_triangle, ray_hit_triangle)
{
    Ray ray(Vector3(1,1,0), Vector3(-1,0,0));
    Triangle triangle(Vector3(0,-40,50), Vector3(0,40,50), Vector3(0,10,-60));
    Intersection i;

    ASSERT_TRUE(triangle.Intersect(ray, i));
    ASSERT_FLOAT_EQ(1, i.t);
}

TEST(ray_triangle, ray_coplanar_triangle)
{
    Ray ray(Vector3(0,0,0), Vector3(1,0,0));
    Triangle triangle(Vector3(10,10,0), Vector3(10,11,0), Vector3(11,10,0));
    Intersection i;

    ASSERT_FALSE(triangle.Intersect(ray, i));
}

TEST(ray_triangle, triangle_behind_ray)
{
    Ray ray(Vector3(0,10,0), Vector3(0,1,0));
    Triangle triangle(Vector3(-1,0,-1), Vector3(-1,0,2), Vector3(2,0,-1));
    Intersection i;

    ASSERT_FALSE(triangle.Intersect(ray, i));
}
