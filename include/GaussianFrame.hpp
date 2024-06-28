#pragma once

#define PCL_NO_PRECOMPILE

#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/point_types.h>
#include <pcl/console/print.h>
#include <pcl/common/transforms.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <bitset>

// Gaussian Point Type
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

typedef struct SH
{
    int index;
    uint8_t r;
    uint8_t g;
    uint8_t b;
    SH():index(0), r(0), g(0), b(0) {}
} SH;

typedef struct Quaternion
{
    int index;
    float q1;
    float q2;
    float q3;
    float q4;
    Quaternion():index(0), q1(0), q2(0), q3(0), q4(0) {}
} Quaternion;

class GaussianFrame
{
public:
    GaussianFrame();
    ~GaussianFrame();

    void readPlyAndTransform(const std::string& filename, float scale, float voxelSize, bool isFlip, float x, float y, float z);
    void generateOctree(float resolution);
    void compressBreadthBytes();
    void reorder();

private:
    int mFrameId;
    int mNumPoints;
    int mOrigNumPoints;

    Eigen::Vector3f mRootCenter;
    float mRootBoxLength;
    int mOctreeDepth;
    int mMaxBreadthDepth;

    pcl::PointCloud<Gaussian>::Ptr mCloud;
    pcl::octree::OctreePointCloud<Gaussian> mOctree;

    std::vector<std::vector<uint8_t>> mBreadthBytes;
    std::vector<uint8_t> mDepthList;
    std::vector<SH> mSHList;
    std::vector<Quaternion> mQuaternionList;
    std::vector<int> mPointIndices;
    std::vector<std::vector<int>> mPartitions;
    
    std::vector<uint8_t> mCompressedDepthBytes;
};
