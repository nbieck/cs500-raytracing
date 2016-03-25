//////////////////////////////////////////////////////////////////////
// Provides the framework for a raytracer.
////////////////////////////////////////////////////////////////////////

#include <vector>

#include <cstdlib>
#include <limits>

#include "geom.h"
#include "raytrace.h"
#include "realtime.h"
#include "minimizer.h"
#include "write_result.h"

#include <gflags/gflags.h>
DEFINE_string(type, "trace",
        "Specify the type of output to be generated. Valid values are:\n"
        "normal -- raycast and output the normal of intersection, output is written to out_base_n.hdr\n"
        "depth -- raycast and output the depth of intersection. output to out_base_d.hdr\n"
        "color -- raycast and output the base color of the hit object, output to out_base_c.hdr\n"
        "lit -- raycast and analytically compute direct lighting, output to out_base_l.hdr\n"
        "trace -- do full pathtrace. This will enter an infinite loop and periodically output "
        "images. Outputs will be written to out_base_#iterations.hdr");
DEFINE_string(out_base, "",
        "Specify the base name for the generated output file. If empty, the name of the "
        "specified scene file will be used (without the .scn) extension");
DEFINE_double(russian_roulette, 0.8, "The russian roulette value that determines whether we continue tracing a ray. Should be between 0 and 1");
DEFINE_int32(dump_rate, 8, "The application will dump images at powers of this number when raytracing");
DEFINE_bool(explicit, true, "Include the explicit light connection in the path tracing algorithm.");
DEFINE_bool(MIS, true, "Include multiple-importance-sampling. Implies explicit light connection.");
DEFINE_bool(AA, true, "Do basic AA by picking ray randomly over full pixel area");
DEFINE_uint64(num_iterations, 0, "The amount of iterations to run, a value of 0 indicates no predefined stopping point.");
DEFINE_bool(overwrite, false, "Keep overwriting the same image instead of making new ones");
DEFINE_bool(reflection, true, "Include specular reflection in the BRDF");

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

// A good quality *thread-safe* Mersenne Twister random number generator.
std::mt19937_64 RNGen;
std::uniform_real_distribution<real> myrandom(0.0, 1.0);
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

        for (auto v : mesh->triangles)
        {
            auto tri = std::make_shared<Triangle>(currentMat);

            tri->m_v0 = mesh->vertices[v[0]].pnt;
            tri->m_v1 = mesh->vertices[v[1]].pnt;
            tri->m_v2 = mesh->vertices[v[2]].pnt;

            tri->m_n0 = mesh->vertices[v[0]].nrm;
            tri->m_n1 = mesh->vertices[v[1]].nrm;
            tri->m_n2 = mesh->vertices[v[2]].nrm;

            objects.push_back(tri);
        }

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
        Matrix4 modelTr = translate(Vector3(f[2],f[3],f[4]))
                          *scale(Vector3(f[5],f[5],f[5]))
                          *toMat4(Orientation(6,strings,f));
        ReadAssimpFile(strings[1], modelTr);  
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
    Minimizer min(ray);
    real mindist = BVMinimize(object_tree, min);

    /*
    for (auto s : objects)
    {
        Intersection test;
        if (s->Intersect(ray, test))
        {
            if (test.t < result.t)
                result = test;
        }
    }
    */

    return min.min_i;
}

Color Scene::Raycast(const Ray& ray, Output output)
{
    Color color = Color(0,0,0);
    Intersection i = CastRay(ray);

    if (std::isinf(i.t))
    {
        color = Color(0,0,0);
    }
    else
    {
        switch (output)
        {
            case Output::Diffuse:
                color = i.obj->mat->Kd;
                break;
            case Output::Depth:
                color = Color(i.t, i.t, i.t);
                break;
            case Output::Normal:
                color = i.n.cwiseAbs();
                break;
            case Output::Position:
                color = i.p;
                break;
            case Output::Lit:
                for (int j = 0; j < lights.size(); ++j)
                {
                    color = color + lights[j]->mat->Kd * i.obj->mat->Kd/PI * i.n.dot((light_pos[j] - i.p).normalized());
                }
                break;
            default:
                break;
        }
    }

    return color;
}

Vector3 SampleLobe(Vector3 N, real cTheta, real Phi)
{
    real sTheta = std::sqrt(1 - cTheta * cTheta);
    Vector3 K = Vector3(sTheta * std::cos(Phi), sTheta * sin(Phi), cTheta);
    Quaternion q = Quaternion::FromTwoVectors(Vector3::UnitZ(), N);
    return q._transformVector(K);
}

real Chi(real d)
{
    if (d > 0)
        return static_cast<real>(1);

    return static_cast<real>(0);
}

//BRDF functions

Color F(real d, const Intersection& i)
{
    return i.obj->mat->Ks + (1 - i.obj->mat->Ks) * std::pow(1 - std::abs(d), 5);
}
real D(const Vector3& m, const Intersection& i)
{
    real m_dot_n = m.dot(i.n);

    if (m_dot_n < std::numeric_limits<real>::epsilon())
        return 0;

    return Chi(m_dot_n) * (i.obj->mat->alpha + 2) / (2 * PI) * std::pow(m_dot_n, i.obj->mat->alpha);
}
real G1(const Vector3& v, const Vector3& m, const Intersection& i)
{
    real v_dot_n = v.dot(i.n);
    if (v_dot_n > (1 - std::numeric_limits<real>::epsilon()))
        return 1;

    if (v_dot_n < std::numeric_limits<real>::epsilon())
        return 1;

    real tan_theta = std::sqrt(1 - v_dot_n * v_dot_n) / v_dot_n;

    if (std::abs(tan_theta) < std::numeric_limits<real>::epsilon())
        return 1;

    real a = std::sqrt(i.obj->mat->alpha / 2 + 1) / tan_theta;

    real val;
    if (a < 1.6)
    {
        val = (3.535 * a + 2.181 * a * a) / (1 + 2.276 * a + 2.577 * a * a);
    }
    else 
        val = 1;

    return Chi(v.dot(m) / v_dot_n) * val;
}
real G(const Vector3& w_o, const Vector3& w_i, const Vector3& m, const Intersection& i)
{
    return G1(w_o, m, i) * G1(w_i, m, i);
}

Vector3 SampleBRDF(const Vector3& w_o, const Intersection& i)
{
    real choice = myrandom(RNGen);

    if (choice < 0.5 || !FLAGS_reflection)
        return SampleLobe(i.n, std::sqrt(myrandom(RNGen)), 2.0 * PI * myrandom(RNGen));
    else
    {
        Vector3 m = SampleLobe(i.n, std::pow(myrandom(RNGen), 1 / (i.obj->mat->alpha + 1)), 2.0 * PI * myrandom(RNGen));
        return 2 * (w_o.dot(m)) * m - w_o;
    }
}

real PDF_BRDF(const Vector3& w_o, const Intersection& i, const Vector3& w_i)
{
    //probability for diffuse
    real P_d = i.n.dot(w_i) / PI;

    if (!FLAGS_reflection)
        return P_d;

    //probablility of reflection
    Vector3 m = (w_o + w_i).normalized();
    if (std::abs(w_i.dot(m)) < std::numeric_limits<real>::epsilon())
            return P_d;

    real P_r = D(m, i) * std::abs(m.dot(i.n)) * (1 / (4 * std::abs(w_i.dot(m))));

    return 0.5 * P_d + 0.5 * P_r;
}

Color EvalBRDF(const Vector3& w_o, const Intersection& i, const Vector3& w_i)
{
    Color diff = i.obj->mat->Kd / PI;

    if (!FLAGS_reflection)
        return diff;

    Vector3 m = (w_o + w_i).normalized();
    real i_dot_n = w_i.dot(i.n);
    real o_dot_n = w_o.dot(i.n);
    if (std::abs(i_dot_n) < std::numeric_limits<real>::epsilon() || std::abs(o_dot_n) < std::numeric_limits<real>::epsilon())
        return 0.5 * diff;

    Color spec = D(m, i) * G(w_o, w_i, m, i) * F(w_i.dot(m), i) / (4 * std::abs(i_dot_n) * std::abs(o_dot_n));

    return 0.5 * diff + 0.5 * spec;
}

Intersection Scene::SampleLight()
{
    std::uniform_int_distribution<int> dist(0, lights.size() - 1);

    int light = dist(RNGen);
    return lights[light]->sample();
}

real Scene::PDFLight(const Intersection& i)
{
    return static_cast<real>(1) / (i.obj->surface() * lights.size());
}

real GeometryTerm(const Intersection& a, const Intersection& b)
{
    Vector3 D = a.p - b.p;
    return std::abs((a.n.dot(D) * b.n.dot(D)) / (D.dot(D) * D.dot(D)));
}

Color Scene::Pathtrace(const Ray& ray)
{
    Color result = Color(0,0,0);
    Color weight = Color(1,1,1);

    Intersection i = CastRay(ray);
    if (std::isinf(i.t))
        return Color(0,0,0);
    if (i.obj->mat->isLight())
    {
        return i.obj->mat->Kd;
    }

    Vector3 w_o = -ray.GetDir();

    while (myrandom(RNGen) < FLAGS_russian_roulette)
    {
        //explicit light goes here
        if (FLAGS_explicit)
        {
            Intersection L = SampleLight();
            real p = PDFLight(L) / GeometryTerm(i,L);
            Vector3 w_i = (L.p - i.p).normalized();
            Intersection LI = CastRay(Ray(i.p, w_i));
            if (p > std::numeric_limits<real>::epsilon() && !std::isinf(LI.t) && LI.obj == L.obj && LI.p.isApprox(L.p))
            {
                Color f = (i.n.dot(w_i)) * EvalBRDF(w_o, i, w_i);
                real MIS = static_cast<real>(1);
                if (FLAGS_MIS)
                {
                    real q = PDF_BRDF(w_o, i, w_i) * FLAGS_russian_roulette;
                    MIS = p * p / (p * p + q * q);
                }
                result += weight * f/p * LI.obj->mat->Kd * MIS;
            }
        }
        
        //keep tracing
        Vector3 w_i = SampleBRDF(w_o, i);
        Intersection i2 = CastRay(Ray(i.p, w_i));
        if (std::isinf(i2.t))
            return result;

        Color f = i.n.dot(w_i) * EvalBRDF(w_o, i, w_i);
        real p = PDF_BRDF(w_o, i,w_i) * FLAGS_russian_roulette;
        if (p < std::numeric_limits<real>::epsilon())
            return result;

        weight *= f/p;

        if (i2.obj->mat->isLight())
        {
            real MIS = static_cast<real>(1);
            if (FLAGS_MIS)
            {
                real q = PDFLight(i2) / GeometryTerm(i,i2);
                MIS = p*p/(p*p+q*q);
            }
            result += MIS * weight * i2.obj->mat->Kd;
            return result;
        }

        i = i2;
        w_o = -w_i;
    }

    return result;
}

void Scene::TraceImage(Color* image, const int pass)
{
    if (FLAGS_MIS)
        FLAGS_explicit = true;

    cam.SetDims(width, height);

    Output o = Output::Trace;
    if (FLAGS_type == "normal")
        o = Output::Normal;
    else if (FLAGS_type == "depth")
        o = Output::Depth;
    else if (FLAGS_type == "color")
        o = Output::Diffuse;
    else if (FLAGS_type == "lit")
        o = Output::Lit;
    else if (FLAGS_type == "trace")
        o = Output::Trace;
    else
        std::cerr << "Unknown value for option \"type\"" << std::endl;

    object_tree = Eigen::KdBVH<real, 3, std::shared_ptr<Shape>>(objects.begin(), objects.end());

    int iterations = 0;
    int output_it = 1;
    //infinite loop for fun and profit
    while (true)
    {
        iterations++;
#pragma omp parallel for schedule(dynamic, 1) // Magic: Multi-thread y loop
        for (int y=0;  y<height;  y++) 
        {
            for (int x=0;  x<width;  x++) 
            {
                Color color(0,0,0);
                
                Ray r = Ray(Vector3(), Vector3());
                if (o != Output::Trace || !FLAGS_AA)
                   r = cam.MakeRay(x, y);
                else 
                   r = cam.MakeRayAA(x, y);

                if (o != Output::Trace)
                    color = Raycast(r, o);
                else 
                    color = Pathtrace(r);

                if (color.allFinite() && (color >= Color(0,0,0)).all())
                    image[y*width + x] += color;
                //Color old = image[y*width + x];
                //image[y*width + x] = old + (1 / static_cast<real>(iterations)) * (color - old);
            }
        }

        //only loop if we are tracing
        if (o != Output::Trace)
            break;
        if (FLAGS_num_iterations != 0 && iterations == FLAGS_num_iterations)
            break;

        if (iterations == output_it)
        {
            std::ostringstream os;
            if (!FLAGS_overwrite)
                os << "_" << iterations;
            WriteHdrImage(FLAGS_out_base + os.str() + ".hdr", width, height, image, static_cast<real>(iterations));
            std::cout << "Writing " + FLAGS_out_base + os.str() + ".hdr" << std::endl;
            if (!FLAGS_overwrite)
                output_it *= FLAGS_dump_rate;
            else
                output_it += FLAGS_dump_rate;
        }
    }

    std::string extension;
    switch (o)
    {
    case Output::Normal:
        extension = "_n";
        break;
    case Output::Depth:
        extension = "_d";
        break;
    case Output::Diffuse:
        extension = "_c";
        break;
    case Output::Lit:
        extension = "_l";
        break;
    case Output::Trace:
        {
            std::ostringstream os;
            os << "_" << iterations;
            extension = os.str();
        }
        break;
    default:
        break;
    }

    WriteHdrImage(FLAGS_out_base + extension + ".hdr", width, height, image, static_cast<real>(iterations));
}
