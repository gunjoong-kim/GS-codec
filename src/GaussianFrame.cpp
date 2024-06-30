#define PCL_NO_PRECOMPILE
#include "GaussianFrame.hpp"

bool cmpColor(const SH& p1, const SH& p2){       

    if(sqrt(p1.r*p1.r + p1.g*p1.g + p1.b*p1.b) < sqrt(p2.r*p2.r + p2.g*p2.g + p2.b*p2.b)) 
    {
        return true;                                     
    }
    return false;
}

uint8_t getDepthFourBits(uint8_t val)                  
{ 

    uint8_t mapped = 0; 
    switch(val)
    {
        case 1:                                           
            mapped = 1;
        case 2:
            mapped = 2;
            break;
        case 4:
            mapped = 3;
            break;
        case 8:
            mapped = 4;                                    
            break;                                            
        case 16:                                          
            mapped = 5;
            break;        
        case 32:
            mapped = 6;
            break;
        case 64:
            mapped = 7;
            break;
        case 128:
            mapped = 8;
            break;
        default:
            printf("ERROR %d\n", val);
            break;
    }
    return mapped;
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
            mBreadthBytes.at(curDepth).push_back(cur.getNodeConfiguration());
        }
        if (curDepth == mMaxBreadthDepth)
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
     for(int i = 0; i < mOctreeDepth; i++)
     {
         std::vector<int> temp;
         mPartitions.push_back(temp);
     }

     int cntChild = 0;
     for(int i = 0 ; i < mBreadthBytes.at(mOctreeDepth - 1).size() ; i++)
     { 
         mPartitions.at(mOctreeDepth - 1).push_back(cntChild);
         std::bitset<8> bs(mBreadthBytes.at(mOctreeDepth - 1).at(i));
         cntChild += bs.count();

     }
     mPartitions.at(mOctreeDepth - 1).push_back(cntChild);
     int leafNodeNum = cntChild; 
     for(int i = mOctreeDepth - 2 ; i >= 0 ; i--)
     { 
         int cntChild = 0; 
         for(int j = 0 ; j < mBreadthBytes.at(i).size() ; j++)
         {
             std::bitset<8> bs(mBreadthBytes.at(i).at(j));
             int childIdx = mPartitions.at(i+1).at(cntChild);
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

    for (int i = 0; i < mPointIndices.size(); i++)
    {
        SH sh;
        Quaternion quaternion;
        sh.index = i;
        sh.r = mCloud->points[mPointIndices[i]].r;
        sh.g = mCloud->points[mPointIndices[i]].g;
        sh.b = mCloud->points[mPointIndices[i]].b;
        reorderedSH.push_back(sh);

        quaternion.index = i;
        quaternion.q1 = mCloud->points[mPointIndices[i]].q1;
        quaternion.q2 = mCloud->points[mPointIndices[i]].q2;
        quaternion.q3 = mCloud->points[mPointIndices[i]].q3;
        quaternion.q4 = mCloud->points[mPointIndices[i]].q4;
        quaternions.push_back(quaternion);
    }

    mSHList = reorderedSH;
    mQuaternionList = quaternions;

    // for (int i = 0; i < numBreadthNodes; i++)
    // {
    //     int startIndex = mPartitions.at(mMaxBreadthDepth).at(i);
    //     int endIndex = mPartitions.at(mMaxBreadthDepth).at(i + 1);
    //     for (int j = startIndex; j < endIndex; j++)
    //     {
    //         SH sh;
    //         Quaternion quaternion;
    //         int pointIndex = mPointIndices[j];

    //         sh.index = j;
    //         sh.r = mCloud->points[pointIndex].r;
    //         sh.g = mCloud->points[pointIndex].g;
    //         sh.b = mCloud->points[pointIndex].b;
    //         reorderedSH.push_back(sh);

    //         quaternion.index = j;
    //         quaternion.q1 = mCloud->points[pointIndex].q1;
    //         quaternion.q2 = mCloud->points[pointIndex].q2;
    //         quaternion.q3 = mCloud->points[pointIndex].q3;
    //         quaternion.q4 = mCloud->points[pointIndex].q4;
    //         quaternions.push_back(quaternion);
    //     }

    //     sort(reorderedSH.begin() + startIndex, reorderedSH.begin() + endIndex, cmpColor);

    //     for (int j = startIndex; j < endIndex; j++)
    //     {
    //         int currentIdx = reorderedSH[j].index;
    //         reorderedDepthNodes.push_back(mDepthList[currentIdx * 3]);
    //         reorderedDepthNodes.push_back(mDepthList[currentIdx * 3 + 1]);
    //         reorderedDepthNodes.push_back(mDepthList[currentIdx * 3 + 2]);
    //         reorderedQuaternions.push_back(quaternions[currentIdx]);
    //     }
    // }

    //mDepthList = reorderedDepthNodes;
    //mQuaternionList = reorderedQuaternions;
    //mSHList = reorderedSH;

    // print size
    std::cout << "mDepthList size: " << mDepthList.size() << std::endl;
    std::cout << "mSHList size: " << mSHList.size() << std::endl;
    std::cout << "mQuaternionList size: " << mQuaternionList.size() << std::endl;

    // //for debug
    // std::cout << "--------------Reordered SH---------------" << std::endl;
    // for (int i = 0; i < mSHList.size(); i++)
    // {
    //     std::cout << "index: " << mSHList[i].index << " r: " << (int)mSHList[i].r << " g: " << (int)mSHList[i].g << " b: " << (int)mSHList[i].b << std::endl;
    // }
    // std::cout << "-----------------------------------------" << std::endl;
}

void GaussianFrame::compressDepthBytes()
{
	std::vector<uint8_t> compressedDepthList;
	if (mDepthList.size() % 2 != 0)
	{
		mDepthList.push_back(0);
	}
	
	for (int i = 0; i < mDepthList.size() / 2; i++)
	{
		uint8_t firstByte = getDepthFourBits(mDepthList[2 * i]);
		uint8_t secondByte = getDepthFourBits(mDepthList[2 * i + 1]);
		secondByte = secondByte << 4;
		uint8_t compressedByte = firstByte | secondByte;
		compressedDepthList.push_back(compressedByte);
	}
	mDepthList = compressedDepthList;

	printf("--------------Compressed Depth Info---------------\n");
	printf("Compressed Depth List Size: %d\n", mDepthList.size());
	printf("--------------------------------------------------\n");
}

std::vector<uint8_t> GaussianFrame::serializeColor()
{
	std::vector<uint8_t> color;
	for (int i = 0; i < mSHList.size(); i++)
	{
		color.push_back(mSHList[i].r);
		color.push_back(mSHList[i].g);
		color.push_back(mSHList[i].b);
	}
	return color;
}

std::vector<float> GaussianFrame::serializeQuaternion()
{
	std::vector<float> quaternion;
	for (int i = 0; i < mQuaternionList.size(); i++)
	{
		quaternion.push_back(mQuaternionList[i].q1);
		quaternion.push_back(mQuaternionList[i].q2);
		quaternion.push_back(mQuaternionList[i].q3);
		quaternion.push_back(mQuaternionList[i].q4);
	}
	return quaternion;
}

void GaussianFrame::generatePayload()
{
	for (int i = 0; i < mMaxBreadthDepth; i++)
	{
		mPayload.breadthBytes.insert(mPayload.breadthBytes.end(), mBreadthBytes[i].begin(), mBreadthBytes[i].end());
	}

	mPayload.depthBytes = mDepthList;

	int cntBreadthLeafIncides = 0;
	for (int i = 1; i < mBreadthBytes[mMaxBreadthDepth].size(); i++)
	{
		int start = mPartitions[mMaxBreadthDepth][i - 1];
		int end = mPartitions[mMaxBreadthDepth][i];
		mPayload.breadthIndex.push_back(end - start);
		cntBreadthLeafIncides += end - start;
	}
	mPayload.colorBytes = serializeColor();
	mPayload.quaternionBytes = serializeQuaternion();
}

void GaussianFrame::generateHeader()
{
	memset(&mHeader, 0, sizeof(Header));
	mHeader.frameType = 'G';
	mHeader.numPoints = mOrigNumPoints;
	mHeader.numBreadthBytes = mPayload.breadthBytes.size();
	mHeader.numBreadthNodes = mPayload.breadthIndex.size();
	mHeader.numDepthBytes = mPayload.depthBytes.size();
	mHeader.numColorBytes = mPayload.colorBytes.size();
	mHeader.numQuaternionBytes = mPayload.quaternionBytes.size();
	mHeader.numIcpBytes = 0;
	mHeader.numIcpNodes = 0;
	mHeader.rootCenter = mRootCenter;
	mHeader.rootBoxLength = mRootBoxLength;
}

void GaussianFrame::writeFrame(const std::string& filename)
{
    generatePayload();
    generateHeader();

    int totalSize = sizeof(Header) + mPayload.breadthBytes.size() + mPayload.depthBytes.size() + mPayload.colorBytes.size() + mPayload.quaternionBytes.size() * sizeof(float);

    std::ofstream outFile(filename, std::ios::binary);
    if (!outFile) {
        throw std::runtime_error("Unable to open file for writing");
    }

    outFile.write(reinterpret_cast<const char*>(&totalSize), sizeof(totalSize));
    outFile.write(reinterpret_cast<const char*>(&mHeader), sizeof(Header));
    outFile.write(reinterpret_cast<const char*>(mPayload.breadthBytes.data()), mPayload.breadthBytes.size());
    outFile.write(reinterpret_cast<const char*>(mPayload.depthBytes.data()), mPayload.depthBytes.size());
    outFile.write(reinterpret_cast<const char*>(mPayload.breadthIndex.data()), mPayload.breadthIndex.size());
    outFile.write(reinterpret_cast<const char*>(mPayload.colorBytes.data()), mPayload.colorBytes.size());
    outFile.write(reinterpret_cast<const char*>(mPayload.quaternionBytes.data()), mPayload.quaternionBytes.size() * sizeof(float));

    outFile.close();
}

void GaussianFrame::decodeFrame(const std::string& filename)
{
	std::ifstream inFile(filename, std::ios::binary);


	int totalSize = 0;
	inFile.read(reinterpret_cast<char*>(&totalSize), sizeof(totalSize));

	Header header;
	inFile.read(reinterpret_cast<char*>(&header), sizeof(Header));

	// print header
	printf("--------------Header Info---------------\n");
	printf("Frame Type: %c\n", header.frameType);
	printf("Number of Points: %d\n", header.numPoints);
	printf("Number of Breadth Bytes: %d\n", header.numBreadthBytes);
	printf("Number of Breadth Nodes: %d\n", header.numBreadthNodes);
	printf("Number of Depth Bytes: %d\n", header.numDepthBytes);
	printf("Number of Color Bytes: %d\n", header.numColorBytes);
	printf("Number of Quaternion Bytes: %d\n", header.numQuaternionBytes);
	printf("Number of ICP Bytes: %d\n", header.numIcpBytes);
	printf("Number of ICP Nodes: %d\n", header.numIcpNodes);
	printf("Root Center: %f %f %f\n", header.rootCenter.x(), header.rootCenter.y(), header.rootCenter.z());
	printf("Root Box Length: %f\n", header.rootBoxLength);
	printf("-----------------------------------------\n");

	std::vector<uint8_t> breadthBytes(header.numBreadthBytes);
	std::vector<uint8_t> depthBytes(header.numDepthBytes);
	std::vector<uint8_t> breadthIndex(header.numBreadthNodes);
	std::vector<uint8_t> colorBytes(header.numColorBytes);
	std::vector<float> quaternionBytes(header.numQuaternionBytes);

	inFile.read(reinterpret_cast<char*>(breadthBytes.data()), header.numBreadthBytes);
	inFile.read(reinterpret_cast<char*>(depthBytes.data()), header.numDepthBytes);
	inFile.read(reinterpret_cast<char*>(breadthIndex.data()), header.numBreadthNodes);
	inFile.read(reinterpret_cast<char*>(colorBytes.data()), header.numColorBytes);
	inFile.read(reinterpret_cast<char*>(quaternionBytes.data()), header.numQuaternionBytes);

	// print size
	printf("--------------Decoded Info---------------\n");
	printf("Breadth Bytes Size: %d\n", breadthBytes.size());
	printf("Depth Bytes Size: %d\n", depthBytes.size());
	printf("Breadth Index Size: %d\n", breadthIndex.size());
	printf("Color Bytes Size: %d\n", colorBytes.size());
	printf("Quaternion Bytes Size: %d\n", quaternionBytes.size());
	printf("-----------------------------------------\n");
}
