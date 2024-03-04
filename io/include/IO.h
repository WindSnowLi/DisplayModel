#pragma once

#include <vtkActor.h>
#include <vtkOBJReader.h>
#include <vtkPDBReader.h>
#include <vtkPLYReader.h>
#include <vtkPolyDataMapper.h>
#include <vtkSTLReader.h>
#include <vtkSmartPointer.h>

#if _WIN32
#include <corecrt_io.h>
#endif

#include <pcl/io/auto_io.h>
#include <pcl/point_types.h>

#include <unordered_map>

namespace IO {
namespace Model {
    /**
     * @brief 读取PLY模型
     * @param filename 文件路径
     * @return  vtkActor
     */
    template <typename T>
    vtkSmartPointer<vtkActor> ReadModel(const std::string& filename)
    {
        vtkSmartPointer<T> reader = vtkSmartPointer<T>::New();
        reader->SetFileName(filename.c_str());

        // Visualize
        vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputConnection(reader->GetOutputPort());
        vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(mapper);
        return actor;
    }

    /**
     * @brief 读取模型
     * @param filename 文件路径
     * @return  vtkActor
     */
    vtkSmartPointer<vtkActor> Read(const std::string& filename)
    {
        // 截取文件类型
        auto&& pos = filename.find_last_of('.');
        if (pos == std::string::npos) {
            return nullptr;
        }
        auto&& type = filename.substr(pos + 1);

        static std::unordered_map<std::string, std::function<vtkSmartPointer<vtkActor>(const std::string&)>> map = {
            { "ply", ReadModel<vtkPLYReader> },
            { "stl", ReadModel<vtkSTLReader> },
            { "obj", ReadModel<vtkOBJReader> },
            { "pdb", ReadModel<vtkPDBReader> },
        };
        if (map.find(type) == map.end()) {
            return nullptr;
        }
        return map[type](filename);
    }

}; // namespace Model
namespace PC {
    /**
     * @brief 读取点云
     * @param filename 文件路径
     * @return  vtkActor
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr Read(const std::string& filename)
    {
        pcl::PCLPointCloud2 cloud_blob;
        if (pcl::io::load(filename, cloud_blob) < 0) {
            return nullptr;
        }
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromPCLPointCloud2(cloud_blob, *cloud);
        return cloud;
    }

}; // namespace PointCloud
}; // namespace Io