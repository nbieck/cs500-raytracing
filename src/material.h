#pragma once

#include "geom.h"


////////////////////////////////////////////////////////////////////////
// Material: encapsulates a BRDF and communication with a shader.
////////////////////////////////////////////////////////////////////////
class Material
{
 public:
    Vector3 Kd, Ks;
    real alpha;

    virtual bool isLight() { return false; }

    Material()  : Kd(Vector3(1.0, 0.5, 0.0)), Ks(Vector3(1,1,1)), alpha(1.0) {}
    Material(const Vector3 d, const Vector3 s, const real a) 
        : Kd(d), Ks(s), alpha(a) {}
    Material(Material& o) { Kd=o.Kd;  Ks=o.Ks;  alpha=o.alpha; }
};

////////////////////////////////////////////////////////////////////////
// Light: encapsulates a light and communiction with a shader.
////////////////////////////////////////////////////////////////////////
class Light: public Material
{
public:

    Light(const Vector3 e) : Material() { Kd = e; }
    virtual bool isLight() { return true; }
};

