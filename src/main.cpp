///////////////////////////////////////////////////////////////////////
// Provides the framework a raytracer.
//
// Gary Herron
//
// Copyright 2012 DigiPen Institute of Technology
////////////////////////////////////////////////////////////////////////

#include <fstream>
#include <sstream>
#include <vector>
#include <string.h>
#include <ctime>

#ifdef _WIN32
    // Includes for Windows
    #include <windows.h>
    #include <cstdlib>
    #include <limits>
    #include <crtdbg.h>
#else
    // Includes for Linux
    #include <stdlib.h>
    #include <time.h> 
    #include <gflags/gflags.h>
#endif

#include "geom.h"
#include "raytrace.h"

DECLARE_string(out_base);

// Read a scene file by parsing each line as a command and calling
// scene->Command(...) with the results.
void ReadScene(const std::string inName, Scene* scene)
{
    std::ifstream input(inName.c_str());
    if (input.fail()) {
        std::cerr << "File not found: "  << inName << std::endl;
        fflush(stderr);
        exit(-1); }

    // For each line in file
    for (std::string line; getline(input, line); ) {
        std::vector<std::string> strings;
        std::vector<real> reals;
        
        // Parse as parallel lists of strings and reals
        std::stringstream lineStream(line);
        for (std::string s; lineStream >> s; ) { // Parses space-separated strings until EOL
            real f;
            //std::stringstream(s) >> f; // Parses an initial real into f, or zero if illegal
            if (!(std::stringstream(s) >> f)) f = nan(""); // An alternate that produced NANs
            reals.push_back(f);
            strings.push_back(s); }

        if (strings.size() == 0) continue; // Skip blanks lines
        if (strings[0][0] == '#') continue; // Skip comment lines
        
        // Pass the line's data to Command(...)
        scene->Command(strings, reals);
    }

    input.close();
}

////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
    std::string usage = "Takes a scene definition file and produces a raytraced image.";

    google::SetUsageMessage(usage);
    google::ParseCommandLineFlags(&argc, &argv, true);

    if (argc < 2)
    {
        std::cout << "Please provide a scene file to render" << std::endl;
        return 0;
    }
    Scene* scene = new Scene();

    // Read the command line argument
    std::string inName =  argv[1];
    std::string hdrName = inName.substr(0, inName.rfind(".scn"));

    google::SetCommandLineOptionWithMode("out_base", hdrName.c_str(), google::SET_FLAG_IF_DEFAULT);

    // Read the scene, calling scene.Command for each line.
    ReadScene(inName, scene);

    scene->Finit();

    // Allocate and clear an image array
    Color *image =  new Color[scene->width*scene->height];
    for (int y=0;  y<scene->height;  y++)
        for (int x=0;  x<scene->width;  x++)
            image[y*scene->width + x] = Color(0,0,0);

    // RayTrace the image
    scene->TraceImage(image, 1);

    delete scene;
    delete[] image;
}
