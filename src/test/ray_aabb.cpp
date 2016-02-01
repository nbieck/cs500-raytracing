#include <gtest/gtest.h>

#include "../ray_shapes.h"

TEST(ray_aabb_intersection, ray_behind_aabb_pointing_back)
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

TEST(ray_aabb_intersection, ray_parallel_to_aabb_side_outside)
{
    Ray ray(Vector3(-3,3,0), Vector3(1,0,0));
    AABB aabb(Vector3(0,0,0), Vector3(1,1,2));
    Intersection i;

    ASSERT_FALSE(aabb.Intersect(ray, i));
}

TEST(ray_aabb_intersection, ray_parallel_to_aabb_side_inside)
{
    Ray ray(Vector3(0.5,4,0.5), Vector3(0,-1,0));
    AABB aabb(Vector3(-1,-1,-1), Vector3(2,3,2));
    Intersection i;

    ASSERT_TRUE(aabb.Intersect(ray, i));
    ASSERT_FLOAT_EQ(2, i.t);
}

TEST(ray_aabb_intersection, ray_coplanar_aabb_side)
{
    Ray ray(Vector3(0,1,-4), Vector3(0,0,1));
    AABB aabb(Vector3(-1,-1,-1), Vector3(2,2,2));
    Intersection i;

    ASSERT_FALSE(aabb.Intersect(ray, i));
}

TEST(ray_aabb_intersection, ray_colinear_aabb_edge)
{
    Ray ray(Vector3(1,1,-3), Vector3(0,0,1));
    AABB aabb(Vector3(-1,-1,-1), Vector3(2,2,2));
    Intersection i;

    ASSERT_FALSE(aabb.Intersect(ray, i));
}

TEST(ray_aabb_intersection, ray_past_aabb_corner)
{
    Ray ray(Vector3(0,-3,3), Vector3(1,1,1));
    AABB aabb(Vector3(-0.5,-0.5,-0.5),Vector3(1,1,1));
    Intersection i;

    ASSERT_FALSE(aabb.Intersect(ray, i));
}

TEST(ray_aabb_intersection, slanted_ray_towards_aabb)
{
    Ray ray(Vector3(2,1,0), Vector3(-1,-1,0));
    AABB aabb(Vector3(-1,-1,-1), Vector3(2,2,2));
    Intersection i;

    ASSERT_TRUE(aabb.Intersect(ray, i));
    ASSERT_FLOAT_EQ(std::sqrt(2), i.t);
}

TEST(ray_aabb_intersection, slanted_ray_away_from_aabb)
{
    Ray ray(Vector3(2,1,0), Vector3(1,1,0));
    AABB aabb(Vector3(-1,-1,-1), Vector3(2,2,2));
    Intersection i;

    ASSERT_FALSE(aabb.Intersect(ray, i));
}

TEST(ray_aabb_intersection, ray_touching_aabb_corner)
{
    Ray ray(Vector3(-1,1,-2), Vector3(1,-1,2));
    AABB aabb(Vector3(0,0,0), Vector3(2,2,2));
    Intersection i;

    ASSERT_FALSE(aabb.Intersect(ray, i));
}

TEST(ray_aabb_intersection, ray_through_aabb_edge)
{
    Ray ray(Vector3(0,-1,-1), Vector3(0,1,1));
    AABB aabb(Vector3(-1,0,0), Vector3(2,1,1));
    Intersection i;

    ASSERT_TRUE(aabb.Intersect(ray, i));
    ASSERT_FLOAT_EQ(std::sqrt(2), i.t);
}

TEST(ray_aabb_intersection, ray_through_aabb_corner)
{
    Ray ray(Vector3(-1,-2,-1), Vector3(1,2,1));
    AABB aabb(Vector3(0,0,0), Vector3(1,1,1));
    Intersection i;

    ASSERT_TRUE(aabb.Intersect(ray, i));
}

TEST(ray_aabb_intersection, ray_starting_on_aabb_pointing_away)
{
    Ray ray(Vector3(0,1,0),Vector3(1,1,-1));
    AABB aabb(Vector3(-1,-1,-1), Vector3(2,2,2));
    Intersection i;

    ASSERT_FALSE(aabb.Intersect(ray, i));
}

TEST(ray_aabb_intersection, ray_starting_on_aabb_pointing_inwards)
{
    Ray ray(Vector3(0,1,0),Vector3(1,-1,1));
    AABB aabb(Vector3(-1,-1,-1), Vector3(2,2,2));
    Intersection i;

    ASSERT_TRUE(aabb.Intersect(ray, i));
    ASSERT_FLOAT_EQ(sqrt(3), i.t);
}
