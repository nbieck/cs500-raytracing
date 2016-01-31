#pragma once
////////////////////////////////////////////////////////////////////////////////
// Temporary code.  Remove this from your raytracer.  This displays
// the contents of a scene file in realtime in a GLUT window.
////////////////////////////////////////////////////////////////////////////////

#include <string>
#include <fstream>
#include <vector>

#include "geom.h"
#include "raytrace.h"

#include <glbinding/gl/gl.h>
#include <glbinding/Binding.h>
using namespace gl;
#include <freeglut.h>


////////////////////////////////////////////////////////////////////////
// Shader programming class;  Encapsulates a OpenGL Shader.
////////////////////////////////////////////////////////////////////////
class ShaderProgram
{
public:
    int program;
    
    void CreateProgram() { program = glCreateProgram(); }
    void Use() { glUseProgram(program); }
    void Unuse() { glUseProgram(0); }
    void CreateShader(const std::string fname, const GLenum type);
    void LinkProgram();

};

////////////////////////////////////////////////////////////////////////
// Obj: encapsulates objects to be drawn; uses OpenGL's VAOs
////////////////////////////////////////////////////////////////////////
class Obj
{
public:
    MeshData* meshdata;
    Matrix4 modelTR;
    Material* material;
    Vector3 center;
    unsigned int vao;
    Obj(MeshData* m, const Matrix4& tr, Material* b);
    void draw();
    Vector3 Center() { return center; }
};

////////////////////////////////////////////////////////////////////////
// Realtime handles all realtime drawing/interaction
////////////////////////////////////////////////////////////////////////
class Realtime
{
public:
    bool nav;
    char motionkey;
    real speed;

    // Camera/viewing parameters
    Vector3 ambient;
    Vector3 eye;      // Position of eye for viewing scene
    Quaternion orient;   // Represents rotation of -Z to view direction
    real ry;
    real front, back;
    real spin, tilt;
    real cDist;              // Distance from eye to center of scene
    //real lightSpin, lightTilt, lightDist;

    int mouseX, mouseY;
    bool shifted;
    bool leftDown;
    bool middleDown;
    bool rightDown;

    MeshData* sphMesh;
    MeshData* boxMesh;
    MeshData* cylMesh;

    ShaderProgram lighting;

    int width, height;
    void setScreen(const int _width, const int _height) { width=_width;  height=_height; }
    void setCamera(const Vector3& _eye, const Quaternion& _o, const real _ry)
    { eye=_eye; orient=_o; ry=_ry; }
    void setAmbient(const Vector3& _a) { ambient = _a; }
    int setTexture(const int width, const int height, unsigned char* image);
    
    std::vector<Obj*> objs;
    std::vector<Obj*> lights;

    Quaternion ViewQuaternion() {
        Quaternion q = angleAxis((tilt-90.0f)*Radians, Vector3(1,0,0))
                       *orient.conjugate()
                       *angleAxis(spin*Radians, Vector3(0,0,1));
        return q.conjugate();
    }

    Vector3 ViewDirection() {
        return ViewQuaternion().toRotationMatrix() * Vector3(0.0f, 0.0f, -1.0f);
    }

    void DrawScene();
    void ReshapeWindow(int w, int h);
    void KeyboardUp(unsigned char key, int x, int y);
    void KeyboardDown(unsigned char key, int x, int y);
    void MouseButton(int button, int state, int x, int y);
    void MouseMotion(int x, int y);
    
    void sphere(const Vector3 center, const real r, Material* mat);
    void box(const Vector3 base, const Vector3 diag, Material* mat);
    void cylinder(const Vector3 base, const Vector3 axis, const real radius, Material* mat);

    void triangleMesh(MeshData* meshdata) {
        Obj* obj = new Obj(meshdata, Matrix4::Identity(), meshdata->mat);
        objs.push_back(obj);
        if (meshdata->mat->isLight())
            lights.push_back(obj);
    }

    Realtime();
    void run();
};


