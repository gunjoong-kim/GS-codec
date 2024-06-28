#define PCL_NO_PRECOMPILE
#include "GaussianFrame.hpp"

bool cmpColor(const SH& p1, const SH& p2){       

    if(sqrt(p1.r*p1.r + p1.g*p1.g + p1.b*p1.b) < sqrt(p2.r*p2.r + p2.g*p2.g + p2.b*p2.b)) 
    {
        return true;                                     
    }
    return false;
}

GaussianFrame::GaussianFrame()
: mOctree(1.0)
{
    mCloud.reset(new pcl::PointCloud<Gaussian>());
}

GaussianFrame::~GaussianFrame()
{
}

void GaussianFrame::readPlyAndTransform(
    const std::string& filename, 
    float scale, 
    float voxelSize, 
    bool isFlip,
    float x,
    float y,
    float z)
{
    // Try to load the PLY file
    if (pcl::io::loadPLYFile<Gaussian>(filename, *mCloud) == -1) {
        // If loading failed, return false
        std::cerr << "Failed to load PLY file: " << filename << std::endl;
        return ;
    }

    mOrigNumPoints = mCloud->width;

    Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
    transform_1(0, 0) = scale;
    transform_1(1, 1) = scale;
    transform_1(2, 2) = scale;
    if (isFlip) {
        transform_1(1, 1) = -scale;
    }

    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
    transform_2.translation() << x, y, z;
    Eigen::Matrix4f transform = transform_2 * transform_1;

    pcl::transformPointCloud(*mCloud, *mCloud, transform);

    // for debug
    //print first point's value
    std::cout << "x: " << mCloud->points[0].x << std::endl;
    std::cout << "y: " << mCloud->points[0].y << std::endl;
    std::cout << "z: " << mCloud->points[0].z << std::endl;
    std::cout << "r: " << (int)mCloud->points[0].r << std::endl;
    std::cout << "g: " << (int)mCloud->points[0].g << std::endl;
    std::cout << "b: " << (int)mCloud->points[0].b << std::endl;
    std::cout << "q1: " << mCloud->points[0].q1 << std::endl;
    std::cout << "q2: " << mCloud->points[0].q2 << std::endl;
    std::cout << "q3: " << mCloud->points[0].q3 << std::endl;
    std::cout << "q4: " << mCloud->points[0].q4 << std::endl;
    std::cout << "size: " << mCloud->size() << std::endl;
}

void GaussianFrame::generateOctree(float resolution)
{
    mOctree.setResolution(resolution);
    mOctree.setInputCloud(mCloud);
    mOctree.defineBoundingBox();
    mOctree.addPointsFromInputCloud();

    double minX, minY, minZ, maxX, maxY, maxZ;
    mOctree.getBoundingBox(minX, minY, minZ, maxX, maxY, maxZ);
    mRootCenter = Eigen::Vector3f((maxX + minX) / 2, (maxY + minY) / 2, (maxZ + minZ) / 2);
    mRootBoxLength = sqrt(mOctree.getVoxelSquaredSideLen(0));

    mOctreeDepth = mOctree.getTreeDepth();
    mMaxBreadthDepth = mOctreeDepth - 3;

    printf("--------------Octree Info---------------\n");
    printf("Number of Points: %d\n",mCloud->width);
    printf("Maximum octree depth: %d\n", mOctreeDepth);
    printf("Root center: %f %f %f\n",mRootCenter.x(), mRootCenter.y(), mRootCenter.z());
    printf("Root Box Size: %f\n", mRootBoxLength);
    printf("-----------------------------------------\n");
}

void GaussianFrame::compressBreadthBytes()
{
    pcl::octree::OctreePointCloud<Gaussian>::DepthFirstIterator cur = mOctree.depth_begin();
    pcl::octree::OctreePointCloud<Gaussian>::DepthFirstIterator end = mOctree.depth_end();

    // for last three depth
    uint8_t currentFirst = 0;
    uint8_t currentSecond = 0;
    uint8_t currentThird = 0;

    int currentFirstCount = 0;
    int currentSecondCount = 0;
    std::vector<uint8_t> currentFirstConfig;
    std::vector<uint8_t> currentSecondConfig;
    std::vector<int> leafNodeIndices;

    // init mBreadthBytes
    for (int i = 0; i < mOctreeDepth; i++)
    {
        std::vector<uint8_t> tmp;
        mBreadthBytes.push_back(tmp);
    }

    bool isFirst = true;
    Eigen::Vector3f curCenter = mRootCenter;
    float curBoxLength = mRootBoxLength;

    while (cur != end)
    {
        int curDepth = cur.getCurrentOctreeDepth();

        if (curDepth < mOctreeDepth)
        {
            mBreadthBytes.at(curDepth).push_back(cur.getCurrentOctreeDepth());
        }
        else if (curDepth == mMaxBreadthDepth)
        {
            currentFirst = cur.getNodeConfiguration();
            std::bitset<8> bs(currentFirst);
            currentFirstConfig.clear();
            currentFirstCount = 0;
            for (int i = 0; i < 8; i++)
            {
                std::bitset<8> currentBs(0);
                if (bs[i])
                {
                    currentBs.set(i);
                    currentFirstConfig.push_back(static_cast<uint8_t>(currentBs.to_ulong()));
                }
            }
        }
        else if (curDepth == mMaxBreadthDepth + 1)
        {
            currentSecond = cur.getNodeConfiguration();
            currentFirst = currentFirstConfig[currentFirstCount];
            std::bitset<8> bs(currentSecond);
            currentSecondConfig.clear();
            currentSecondCount = 0;
            for (int i = 0; i < 8; i++)
            {
                std::bitset<8> currentBs(0);
                if (bs[i])
                {
                    currentBs.set(i);
                    currentSecondConfig.push_back(static_cast<uint8_t>(currentBs.to_ulong()));
                }
            }
            currentFirstCount++;
        }
        else if (curDepth == mMaxBreadthDepth + 2)
        {
            currentSecond = currentSecondConfig[currentSecondCount];
            currentThird = cur.getNodeConfiguration();
            std::bitset<8> bs(currentThird);
            for (int i = 0; i < 8; i++)
            {
                std::bitset<8> currentBs(0);
                if (bs[i])
                {
                    currentBs.set(i);
                    uint8_t currentLast = static_cast<uint8_t>(currentBs.to_ulong());
                    mDepthList.push_back(currentFirst);
                    mDepthList.push_back(currentSecond);
                    mDepthList.push_back(currentLast);
                }
            }
            currentSecondCount++;
        }
        else if (curDepth == mMaxBreadthDepth + 3)
        {
            // get leaf node indices
            pcl::octree::OctreeLeafNode<pcl::octree::OctreeContainerPointIndices>* leafNode = static_cast<pcl::octree::OctreeLeafNode<pcl::octree::OctreeContainerPointIndices>*>(cur.getCurrentOctreeNode());
            pcl::octree::OctreeContainerPointIndices contatiner = leafNode->getContainer();
            std::vector<int> pointIndices;
            contatiner.getPointIndices(pointIndices);
            mPointIndices.push_back(pointIndices[0]); // because each leaf node only has one point
        }
        cur++;
    }

    // generate leaf node indices
    for (int i = 0; i < mOctreeDepth; i++)
    {
        std::vector<int> tmp;
        mPartitions.push_back(tmp);
    }

    int cntChild = 0;
    for (int i = 0; i < mBreadthBytes.at(mOctreeDepth - 1).size(); i++)
    {
        mPartitions.at(mOctreeDepth - 1).push_back(cntChild);
        std::bitset<8> bs(mBreadthBytes.at(mOctreeDepth - 1).at(i));
        cntChild += bs.count();
    }
    mPartitions.at(mOctreeDepth - 1).push_back(cntChild);

    int leafNodeNum = cntChild;

    printf("mOctreeDepth : %d\n", mOctreeDepth);

    for (int i = mOctreeDepth - 2; i >= 0; i--)
    {
        printf("breadthbytes size : %d\n", mBreadthBytes.at(i).size());
        int cntChild = 0;
        for (int j = 0; j < mBreadthBytes.at(i).size(); j++)
        {
            std::bitset<8> bs(mBreadthBytes.at(i).at(j));
            printf("%d , %d\n", mPartitions.at(i + 1).size(), cntChild);
            int childIdx = mPartitions.at(i + 1).at(cntChild);
            mPartitions.at(i).push_back(childIdx);
            cntChild += bs.count();
        }
        mPartitions.at(i).push_back(leafNodeNum);
    }
}

void GaussianFrame::reorder()
{
    int numBreadthNodes = mBreadthBytes.at(mMaxBreadthDepth).size();
    std::vector<uint8_t> reorderedDepthNodes;
    std::vector<SH> reorderedSH;
    std::vector<Quaternion> quaternions;
    std::vector<Quaternion> reorderedQuaternions;

    for (int i = 0; i < numBreadthNodes; i++)
    {
        int startIndex = mPartitions.at(mMaxBreadthDepth).at(i);
        int endIndex = mPartitions.at(mMaxBreadthDepth).at(i + 1);
        for (int j = startIndex; j < endIndex; j++)
        {
            SH sh;
            Quaternion quaternion;
            int pointIndex = mPointIndices[j];

            sh.index = j;
            sh.r = mCloud->points[pointIndex].r;
            sh.g = mCloud->points[pointIndex].g;
            sh.b = mCloud->points[pointIndex].b;
            reorderedSH.push_back(sh);

            quaternion.index = j;
            quaternion.q1 = mCloud->points[pointIndex].q1;
            quaternion.q2 = mCloud->points[pointIndex].q2;
            quaternion.q3 = mCloud->points[pointIndex].q3;
            quaternion.q4 = mCloud->points[pointIndex].q4;
            quaternions.push_back(quaternion);
        }

        sort(reorderedSH.begin() + startIndex, reorderedSH.begin() + endIndex, cmpColor);

        for (int j = startIndex; j < endIndex; j++)
        {
            int currentIdx = reorderedSH[j].index;
            reorderedDepthNodes.push_back(mDepthList[currentIdx * 3]);
            reorderedDepthNodes.push_back(mDepthList[currentIdx * 3 + 1]);
            reorderedDepthNodes.push_back(mDepthList[currentIdx * 3 + 2]);
            reorderedQuaternions.push_back(quaternions[currentIdx]);
        }
    }

    mDepthList = reorderedDepthNodes;
    mQuaternionList = reorderedQuaternions;
    mSHList = reorderedSH;

    // print size
    std::cout << "mDepthList size: " << mDepthList.size() << std::endl;
    std::cout << "mSHList size: " << mSHList.size() << std::endl;
    std::cout << "mQuaternionList size: " << mQuaternionList.size() << std::endl;

    //for debug
    std::cout << "--------------Reordered SH---------------" << std::endl;
    for (int i = 0; i < mSHList.size(); i++)
    {
        std::cout << "index: " << mSHList[i].index << " r: " << (int)mSHList[i].r << " g: " << (int)mSHList[i].g << " b: " << (int)mSHList[i].b << std::endl;
    }
    std::cout << "-----------------------------------------" << std::endl;
}