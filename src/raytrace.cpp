//////////////////////////////////////////////////////////////////////
// Provides the framework for a raytracer.
////////////////////////////////////////////////////////////////////////

#include <vector>

#include <cstdlib>
#include <limits>

#include "geom.h"
#include "raytrace.h"
#include "realtime.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

// A good quality *thread-safe* Mersenne Twister random number generator.
#include <random>
std::mt19937_64 RNGen;
std::uniform_real_distribution<> myrandom(0.0, 1.0);
// Call myrandom(RNGen) to get a uniformly distributed random number in [0,1].

Scene::Scene() 
    : cam(Vector3(), Quaternion(), 1)
{ 

}

void Scene::Finit()
{
    currentMat = std::make_shared<Material>(Vector3(0,0,0), Vector3(0,0,0), 1);
}

void Scene::triangleMesh(MeshData* mesh) 
{ 

    if (mesh)
    {
        if (mesh->mat)
            delete mesh->mat;
        delete mesh;
    }
}

Quaternion Orientation(int i, 
                        const std::vector<std::string>& strings,
                        const std::vector<real>& f)
{
    Quaternion q(1,0,0,0); // Unit quaternion
    while (i<strings.size()) {
        std::string c = strings[i++];
        if (c == "x")  
            q *= angleAxis(f[i++]*Radians, Vector3::UnitX());
        else if (c == "y")  
            q *= angleAxis(f[i++]*Radians, Vector3::UnitY());
        else if (c == "z")  
            q *= angleAxis(f[i++]*Radians, Vector3::UnitZ());
        else if (c == "q")  {
            q *= Quaternion(f[i+0], f[i+1], f[i+2], f[i+3]);
            i+=4; }
        else if (c == "a")  {
            q *= angleAxis(f[i+0]*Radians, Vector3(f[i+1], f[i+2], f[i+3]).normalized());
            i+=4; } }
    return q;
}

void Scene::Command(const std::vector<std::string> strings,
                    const std::vector<real> f)
{
    if (strings.size() == 0) 
        return;
    std::string c = strings[0];

    if (c == "screen") 
    {
        // syntax: screen width height
        
        width = int(f[1]);
        height = int(f[2]); 
    }
    else if (c == "camera") 
    {
        // syntax: camera x y z   ry   <orientation spec>
        // Eye position (x,y,z),  view orientation (qw qx qy qz),  frustum height ratio ry
        cam = Camera(Vector3(f[1], f[2], f[3]), Orientation(5, strings, f), f[4]);
    }
    else if (c == "ambient") 
    {
        // syntax: ambient r g b
        // Sets the ambient color.  Note: This parameter is temporary.
        // It will be ignored once your raytracer becomes capable of
        // accurately *calculating* the true ambient light.

    }
    else if (c == "brdf")  
    {
        // syntax: brdf  r g b   r g b  alpha
        // later:  brdf  r g b   r g b  alpha  r g b ior
        // First rgb is Diffuse reflection, second is specular reflection.
        // third is beer's law transmission followed by index of refraction.
        // Creates a Material instance to be picked up by successive shapes
        currentMat = std::make_shared<Material>(Vector3(f[1], f[2], f[3]), Vector3(f[4], f[5], f[6]), f[7]);
    }
    else if (c == "light") 
    {
        // syntax: light  r g b   
        // The rgb is the emission of the light
        // Creates a Material instance to be picked up by successive shapes
        currentMat = std::make_shared<Light>(Vector3(f[1], f[2], f[3])); 
    }
    else if (c == "sphere") 
    {
        // syntax: sphere x y z   r
        // Creates a Shape instance for a sphere defined by a center and radius

        auto sphere = std::make_shared<Sphere>(
                Vector3(f[1], f[2], f[3]), 
                f[4], 
                currentMat);

        if (currentMat->isLight())
        {
            lights.push_back(sphere);
            light_pos.push_back(Vector3(f[1],f[2],f[3]));
        }

        objects.push_back(sphere);
    }
    else if (c == "box") 
    {
        // syntax: box bx by bz   dx dy dz
        // Creates a Shape instance for a box defined by a corner point and diagonal vector

        auto aabb = std::make_shared<AABB>(
                Vector3(f[1], f[2], f[3]), 
                Vector3(f[4], f[5], f[6]), 
                currentMat);

        if (currentMat->isLight())
        {
            lights.push_back(aabb);
            light_pos.push_back(Vector3(f[1],f[2],f[3])+Vector3(f[4],f[5],f[6])/2);
        }

        objects.push_back(aabb);
    }
    else if (c == "cylinder") 
    {
        // syntax: cylinder bx by bz   ax ay az  r
        // Creates a Shape instance for a cylinder defined by a base point, axis vector, and radius
        auto cylinder = std::make_shared<Cylinder>(
                Vector3(f[1],f[2],f[3]),
                Vector3(f[4],f[5],f[6]),
                f[7],
                currentMat);
        
        if (currentMat->isLight())
        {
            lights.push_back(cylinder);
            light_pos.push_back(Vector3(f[1],f[2],f[3])+Vector3(f[4],f[5],f[6])/2);
        }

        objects.push_back(cylinder);
    }
    else if (c == "mesh") 
    {
        // syntax: mesh   filename   tx ty tz   s   <orientation>
        // Creates many Shape instances (one per triangle) by reading
        // model(s) from filename. All triangles are rotated by a
        // quaternion (qw qx qy qz), uniformly scaled by s, and
        // translated by (tx ty tz) .
        //Matrix4 modelTr = translate(Vector3(f[2],f[3],f[4]))
        //                  *scale(Vector3(f[5],f[5],f[5]))
        //                  *toMat4(Orientation(6,strings,f));
        //ReadAssimpFile(strings[1], modelTr);  
    }
    else 
    {
        fprintf(stderr, "\n*********************************************\n");
        fprintf(stderr, "* Unknown command: %s\n", c.c_str());
        fprintf(stderr, "*********************************************\n\n");
    }
}

Intersection Scene::CastRay(const Ray& ray)
{
    Intersection result;
    result.t = std::numeric_limits<real>::infinity();

    for (auto s : objects)
    {
        Intersection test;
        if (s->Intersect(ray, test))
        {
            if (test.t < result.t)
                result = test;
        }
    }

    return result;
}

void Scene::TraceImage(Color* image, const int pass)
{

#pragma omp parallel for schedule(dynamic, 1) // Magic: Multi-thread y loop
    for (int y=0;  y<height;  y++) 
    {
        for (int x=0;  x<width;  x++) 
        {
            Color color(0,0,0);
            
            Ray r = cam.MakeRay(width, height, x, y);
            Intersection i = CastRay(r);

            if (std::isinf(i.t))
            {
                color = Color(0,0,0);
            }
            else
            {
                //color = i.obj->mat->Kd;
                //color = Color(i.t, i.t, i.t);
                color = i.n.cwiseAbs();
                //for (int j = 0; j < lights.size(); ++j)
                //{
                //    color = color + lights[j]->mat->Kd * i.obj->mat->Kd/PI * i.n.dot((light_pos[j] - i.p).normalized());
                //}
            }

            image[y*width + x] = color;
        }
    }
}
