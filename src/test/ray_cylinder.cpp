#include <gtest/gtest.h>

#include "../ray_shapes.h"

TEST(ray_cylinder, ray_away_from_cylinder_parallel_to_caps)
{
    Ray ray(Vector3(1,0,0), Vector3(1,0,0));
    Cylinder c(Vector3(0,-1,0),Vector3(0,2,0),0.5);
    Intersection i;

    ASSERT_FALSE(c.Intersect(ray, i));
}

TEST(ray_cylinder, ray_towards_cylinder_parallel_to_caps)
{
    Ray ray(Vector3(1,0,0), Vector3(-1,0,0));
    Cylinder c(Vector3(0,0,-1),Vector3(0,0,1.5),0.5);
    Intersection i;

    ASSERT_TRUE(c.Intersect(ray, i));
    ASSERT_FLOAT_EQ(0.5, i.t);
    ASSERT_EQ(&c, i.obj);
}

TEST(ray_cylinder, ray_inside_cylinder_parallel_to_caps)
{
    Ray ray(Vector3(0,0,0), Vector3(1,1,0));
    Cylinder c(Vector3(0,0,-1), Vector3(0,0,2), 1.4);
    Intersection i;

    ASSERT_TRUE(c.Intersect(ray, i));
    ASSERT_FLOAT_EQ(1.4, i.t);
}

TEST(ray_cylinder, ray_tangent_cylinder_parallel_to_caps)
{
    Ray ray(Vector3(1,1,0), Vector3(0,-1,0));
    Cylinder c(Vector3(0,0,1), Vector3(0,0,-2), 1);
    Intersection i;

    ASSERT_FALSE(c.Intersect(ray, i));
}

TEST(ray_cylinder, ray_past_cylinder)
{
    Ray ray(Vector3(3,1,-5), Vector3(2,5,-0.4));
    Cylinder c(Vector3(4,1,2), Vector3(0.23, 4, 1), 2.3);
    Intersection i;

    ASSERT_FALSE(c.Intersect(ray, i));
}

TEST(ray_cylinder, ray_away_from_cylinder_caps)
{
    Ray ray(Vector3(0,3,3), Vector3(0,1,1));
    Cylinder c(Vector3(0,-1,-1), Vector3(0,2,2), 2);
    Intersection i;

    ASSERT_FALSE(c.Intersect(ray, i));
}

TEST(ray_cylinder, ray_towards_cylinder_caps)
{
    Ray ray(Vector3(0,3,3), Vector3(0,-1,-1));
    Cylinder c(Vector3(0,-1,-1), Vector3(0,2,2), 3);
    Intersection i;

    ASSERT_TRUE(c.Intersect(ray, i));
}

TEST(ray_cylinder, ray_inside_cylinder_parallel_to_axis)
{
    Ray ray(Vector3(0,0,0), Vector3(1,3,2));
    Cylinder c(Vector3(-1,4,2), Vector3(3,9,6), 5);
    Intersection i;

    ASSERT_TRUE(c.Intersect(ray, i));
}

TEST(ray_cylinder, ray_outside_cylinder_parallel_to_axis)
{
    Ray ray(Vector3(0,0,0), Vector3(0,0,1));
    Cylinder c(Vector3(3,3,0), Vector3(0,0,1), 2);
    Intersection i;

    ASSERT_FALSE(c.Intersect(ray, i));
}

TEST(ray_cylinder, ray_on_cylinder_side_pointing_out)
{
    Ray ray(Vector3(0,1,0), Vector3(1,1,0));
    Cylinder c(Vector3(0,0,-3), Vector3(0,0,5), 1);
    Intersection i;

    ASSERT_FALSE(c.Intersect(ray, i));
}

TEST(ray_cylinder, ray_on_cylinder_side_pointing_in)
{
    Ray ray(Vector3(0,1,0), Vector3(0,-1,0));
    Cylinder c(Vector3(-1,0,-1), Vector3(2,0,2), 1);
    Intersection i;

    ASSERT_TRUE(c.Intersect(ray, i));
    ASSERT_FLOAT_EQ(2, i.t);
}

TEST(ray_cylinder, ray_on_cylinder_cap_pointing_out)
{
    Ray ray(Vector3(0,1,1), Vector3(0,1,1));
    Cylinder c(Vector3(0,0,0), Vector3(0,1,1), 2);
    Intersection i;

    ASSERT_FALSE(c.Intersect(ray, i));
}

TEST(ray_cylinder, ray_on_cylinder_cap_pointing_in)
{
    Ray ray(Vector3(2,2,2), Vector3(-1,-2,-1));
    Cylinder c(Vector3(0,0,0), Vector3(2,2,2), 4);
    Intersection i;

    ASSERT_TRUE(c.Intersect(ray, i));
}

TEST(ray_cylinder, ray_touching_cylinder_edge)
{
    Ray ray(Vector3(2,0,0), Vector3(-1,1,0));
    Cylinder c(Vector3(0,0,0), Vector3(0,1,0), 1);
    Intersection i;

    ASSERT_FALSE(c.Intersect(ray, i));
}
