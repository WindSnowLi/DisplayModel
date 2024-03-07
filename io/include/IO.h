#pragma once

#include <vtkActor.h>
#include <vtkPolyDataMapper.h>
#include <vtkSmartPointer.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

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
        vtkNew<T> reader;
        reader->SetFileName(filename.c_str());
        vtkNew<vtkActor> actor;

        vtkNew<vtkPolyDataMapper> mapper;
        mapper->SetInputConnection(reader->GetOutputPort());

        actor->SetMapper(mapper);

        reader->Update();

        return actor;
    }

    /**
     * @brief 读取SLC模型
     * @param filename 文件路径
     * @param isoValue 等值面值
     * @return  vtkActor
     */
    vtkSmartPointer<vtkActor> ReadSLC(const std::string& filename, double isoValue = 72);

    /**
     * @brief 读取STL模型
     * @param filename 文件路径
     * @return  vtkActor
     */
    vtkSmartPointer<vtkActor> ReadIGES(const std::string& filename);

    /**
     * @brief 读取STL模型
     * @param filename 文件路径
     * @return  vtkActor
     */
    vtkSmartPointer<vtkActor> ReadIGESOnMeshes(const std::string& filename);

    /**
     * @brief 读取STL模型
     * @param filename 文件路径
     * @return  vtkActor
     */
    vtkSmartPointer<vtkActor> ReadSTEP(const std::string& filename);

    /**
     * @brief 读取模型
     * @param filename 文件路径
     * @return  vtkActor
     */
    vtkSmartPointer<vtkActor> Read(const std::string& filename);

}; // namespace Model
namespace PC {
    /**
     * @brief 读取点云
     * @param filename 文件路径
     * @return  vtkActor
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr Read(const std::string& filename);

}; // namespace PointCloud
}; // namespace IO