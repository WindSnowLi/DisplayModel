#include "IO.h"

pcl::PointCloud<pcl::PointXYZRGB>::Ptr IO::PC::Read(const std::string& filename)
{
    pcl::PCLPointCloud2 cloud_blob;
    if (pcl::io::load(filename, cloud_blob) < 0) {
        return nullptr;
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(cloud_blob, *cloud);
    return cloud;
}