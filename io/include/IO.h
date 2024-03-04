#pragma once

#include <vtkActor.h>
#include <vtkSmartPointer.h>

namespace IO {
namespace Model {
    /**
     * @brief 读取模型
     * @param filename 文件路径
     * @return  vtkActor
     */
    vtkSmartPointer<vtkActor> Read(const std::string& filename);

    /**
     * @brief 读取PLY模型
     * @param filename 文件路径
     * @return  vtkActor
     */
    vtkSmartPointer<vtkActor> ReadPLY(const std::string& filename);

    /**
     * @brief 读取STL模型
     * @param filename 文件路径
     * @return  vtkActor
     */
    vtkSmartPointer<vtkActor> ReadSTL(const std::string& filename);

    /**
     * @brief 读取OBJ模型
     * @param filename 文件路径
     * @return  vtkActor
     */
    vtkSmartPointer<vtkActor> ReadOBJ(const std::string& filename);

    /**
     * @brief 读取PDB模型
     * @param filename 文件路径
     * @return  vtkActor
     */
    vtkSmartPointer<vtkActor> ReadPDB(const std::string& filename);

}; // namespace Model
}; // namespace Io