#define PCL_NO_PRECOMPILE
#include <GaussianCodec.hpp>

int main(int argc, char** argv)
{
    // if (argc != 2)
    // {
    //     std::cerr << "Usage: " << argv[0] << " <input.ply>" << std::endl;
    //     return -1;
    // }

    float scale = 1;
    float voxelSize = 0.0001;
    bool isFlip = true;
    float x = 0.0;
    float y = 0.0;
    float z = 0.0;

    GaussianFrame frame;
    frame.readPlyAndTransform("../data/mean_color_quaternion.ply", scale, voxelSize, isFlip, x, y, z);
    frame.generateOctree(voxelSize);
    frame.compressBreadthBytes();
    frame.reorder();
    return 0;
}