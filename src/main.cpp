#define PCL_NO_PRECOMPILE
#include <GaussianCodec.hpp>

int main(int argc, char** argv)
{
    // if (argc < 2)
    // {
    //     std::cerr << "Usage: " << argv[0] << " <input.ply>" << std::endl;
    //     return -1;
    // }

    GaussianCodec codec;
    codec.readGaussianPly("../../pointcloud/dgs/mean_color_quaternion.ply");

    // compress the point cloud
    codec.encodeGaussian();
    return 0;
}