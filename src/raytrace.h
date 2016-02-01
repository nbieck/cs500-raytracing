#pragma once
///////////////////////////////////////////////////////////////////////
// A framework for a raytracer.
////////////////////////////////////////////////////////////////////////

#include "geom.h"
#include "material.h"
#include "ray_shapes.h"
#include "camera.h"

#include <memory>
#include <vector>

class Shape;

const real PI = 3.14159f;
const real Radians = PI/180.0f;    // Convert degrees to radians

////////////////////////////////////////////////////////////////////////
// Data structures for storing meshes -- mostly used for model files
// read in via ASSIMP.
//
// A MeshData holds two lists (stl::vector) one for vertices
// (VertexData: consisting of point, normal, texture, and tangent
// vectors), and one for triangles (TriData: consisting of three
// indices into the vertex array).
typedef Eigen::Matrix<unsigned int, 3, 1 > TriData;
    
class VertexData
{
 public:
    Vector3 pnt;
    Vector3 nrm;
    Vector2 tex;
    Vector3 tan;
    VertexData(const Vector3& p, const Vector3& n, const Vector2& t, const Vector3& a) 
        : pnt(p), nrm(n), tex(t), tan(a) 
    {}
};

struct MeshData
{
    std::vector<VertexData> vertices;
    std::vector<TriData> triangles;
    Material *mat;
};

////////////////////////////////////////////////////////////////////////////////
// Scene
class Realtime;

class Scene 
{
public:
    int width, height;
//    Realtime* realtime;         // Remove this (realtime stuff)
    std::shared_ptr<Material> currentMat;
    std::vector<std::shared_ptr<Shape>> objects;
    std::vector<std::shared_ptr<Shape>> lights;
    Camera cam;

    Scene();
    void Finit();

    // The scene reader-parser will call the Command method with the
    // contents of each line in the scene file.
    void Command(const std::vector<std::string> strings,
                 const std::vector<real> f);

    // To read a model file into the scene via ASSIMP, call ReadAssimpFile.  
    void ReadAssimpFile(const std::string& path, const Matrix4& M);

    // Once ReadAssimpFile parses the information from the model file,
    // it will call:
    void triangleMesh(MeshData* mesh);

    //casts a ray into the scene and returns the closest intersection
    //t value will be infinite, if no intersection is found
    Intersection CastRay(const Ray& ray);

    // The main program will call the TraceImage method to generate
    // and return the image.  This is the Ray Tracer!
    void TraceImage(Color* image, const int pass);
};
