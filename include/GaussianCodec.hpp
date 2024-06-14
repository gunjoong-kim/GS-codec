#ifndef GAUSSIAN_CODEC
#define GAUSSIAN_CODEC

#define PCL_NO_PRECOMPILE

#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/point_types.h>
#include <pcl/console/print.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <fstream>

// Gaussian Data Type
struct EIGEN_ALIGN16 Gaussian
{
    PCL_ADD_POINT4D;
    PCL_ADD_RGB;
    float q1;
    float q2;
    float q3;
    float q4;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(Gaussian,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (uint8_t, r, r)
    (uint8_t, g, g)
    (uint8_t, b, b)
    (float, q1, q1)
    (float, q2, q2)
    (float, q3, q3)
    (float, q4, q4)
)


class GaussianCodec
{
public:
    GaussianCodec();
    ~GaussianCodec();

    void readGaussianPly(const std::string& filename);
    void encodeGaussian();
    void decodeGaussian();

public:
    pcl::PointCloud<Gaussian>::Ptr m_cloud;

    pcl::io::OctreePointCloudCompression<Gaussian> m_encoder;
    pcl::io::OctreePointCloudCompression<Gaussian> m_decoder;

// for debug
public:
    void visualizeGaussian();

};

#endif