#pragma once

#include "geom.h"
#include "ray_shapes.h"

class Camera
{
public:

    Camera(const Vector3& eye, const Quaternion& q, real ratio);

    Ray MakeRay(int width, int height, int x, int y);

private:
    
    Vector3 m_eye;
    Quaternion m_q;
    real m_ratio;

    Vector3 X, Y, Z;
};
