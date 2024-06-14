#define PCL_NO_PRECOMPILE
#include <GaussianCodec.hpp>

GaussianCodec::GaussianCodec()
{
    m_cloud.reset(new pcl::PointCloud<Gaussian>());
}

GaussianCodec::~GaussianCodec()
{
    // smart pointer will automatically delete the memory
}

void GaussianCodec::readGaussianPly(const std::string& filename)
{
    // Try to load the PLY file
    if (pcl::io::loadPLYFile<Gaussian>(filename, *m_cloud) == -1) {
        // If loading failed, return false
        std::cerr << "Failed to load PLY file: " << filename << std::endl;
        return ;
    }

    // for debug
    //print first point's value
    std::cout << "x: " << m_cloud->points[0].x << std::endl;
    std::cout << "y: " << m_cloud->points[0].y << std::endl;
    std::cout << "z: " << m_cloud->points[0].z << std::endl;
    std::cout << "r: " << (int)m_cloud->points[0].r << std::endl;
    std::cout << "g: " << (int)m_cloud->points[0].g << std::endl;
    std::cout << "b: " << (int)m_cloud->points[0].b << std::endl;
    std::cout << "q1: " << m_cloud->points[0].q1 << std::endl;
    std::cout << "q2: " << m_cloud->points[0].q2 << std::endl;
    std::cout << "q3: " << m_cloud->points[0].q3 << std::endl;
    std::cout << "q4: " << m_cloud->points[0].q4 << std::endl;
    std::cout << "size: " << m_cloud->size() << std::endl;
}

void GaussianCodec::encodeGaussian()
{
    std::stringstream compressedData;
    m_encoder.encodePointCloud(m_cloud, compressedData);

    std::string output_filename = "./compressed_data.bin";
    std::ofstream outFile(output_filename, std::ios::out | std::ios::binary);
    //outFile << compressedData.rdbuf();
    //outFile.close();
}

void GaussianCodec::decodeGaussian()
{
    // std::string input_filename = "./compressed_data.bin";
    // std::ifstream inFile(input_filename, std::ios::in | std::ios::binary);

    // std::stringstream compressedData;
    // // 파일 내용을 스트림으로 복사
    // compressedData << inFile.rdbuf();
    // inFile.close();
    // pcl::io::OctreePointCloudCompression<Gaussian>* pointCloudDecoder;
    // pointCloudDecoder = new pcl::io::OctreePointCloudCompression<Gaussian>();

    // // 포인트 클라우드 객체를 생성합니다.
    // pcl::PointCloud<Gaussian>::Ptr cloud(new pcl::PointCloud<Gaussian>);

    // // 압축 해제
    // pointCloudDecoder->decodePointCloud(compressedData, cloud);
}

void GaussianCodec::visualizeGaussian()
{
    // pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Gaussian Cloud"));
    // viewer->setBackgroundColor(0.0, 0.0, 0.0);
    // viewer->addPointCloud<Gaussian>(m_cloud, "Gaussian Cloud");
    // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Gaussian Cloud");
    // viewer->addCoordinateSystem(1.0);
    // viewer->initCameraParameters();

    // while (!viewer->wasStopped()) {
    //     viewer->spinOnce(100);
    // }
}