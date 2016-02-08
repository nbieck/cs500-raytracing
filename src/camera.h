#pragma once

#include "geom.h"
#include "ray_shapes.h"

class Camera
{
public:

    Camera(const Vector3& eye, const Quaternion& q, real ratio);

    void SetDims(int width, int height);

    Ray MakeRay(int x, int y);
    Ray MakeRayAA(int x, int y);

private:
    
    Vector3 m_eye;
    Quaternion m_q;
    real m_ratio;
    real ratio_x;
    int m_width, m_height;

    Vector3 X, Y, Z;
};
