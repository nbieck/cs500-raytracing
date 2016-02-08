#include "write_result.h"

#include <sys/stat.h>
#include <sys/types.h>

void MakePath(const std::string outName)
{
    size_t dir_loc = outName.find_first_of("/\\");
    while (dir_loc != std::string::npos)
    {
        mkdir(outName.substr(0, dir_loc).c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
        dir_loc = outName.find_first_of("/\\", dir_loc + 1);
    }
}

// Write the image as a HDR(RGBE) image.  
#include "rgbe.h"
void WriteHdrImage(const std::string outName, const int width, const int height, Color* image)
{
    // Turn image from a 2D-bottom-up array of Vector3D to an top-down-array of reals
    real* data = new real[width*height*3];
    real* dp = data;
    for (int y=height-1;  y>=0;  --y) {
        for (int x=0;  x<width;  ++x) {
            Color pixel = image[y*width + x];
            *dp++ = pixel[0];
            *dp++ = pixel[1];
            *dp++ = pixel[2]; } }

    // Write image to file in HDR (a.k.a RADIANCE) format
    rgbe_header_info info;
    char errbuf[100] = {0};

    MakePath(outName);
    FILE* fp  =  fopen(outName.c_str(), "wb");
    info.valid = false;
    int r = RGBE_WriteHeader(fp, width, height, &info, errbuf);
    if (r != RGBE_RETURN_SUCCESS)
        printf("error: %s\n", errbuf);

    r = RGBE_WritePixels_RLE(fp, data, width,  height, errbuf);
    if (r != RGBE_RETURN_SUCCESS)
        printf("error: %s\n", errbuf);
    fclose(fp);
    
    delete[] data;
}

