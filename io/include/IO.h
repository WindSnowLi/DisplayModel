#pragma once

#include <vtk3DSImporter.h>
#include <vtkActor.h>
#include <vtkCMLMoleculeReader.h>
#include <vtkExodusIIReader.h>
#include <vtkGLTFImporter.h>
#include <vtkMultiBlockPLOT3DReader.h>
#include <vtkOBJImporter.h>
#include <vtkOBJReader.h>
#include <vtkPDBReader.h>
#include <vtkPLYReader.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkRendererCollection.h>
#include <vtkSLCReader.h>
#include <vtkSTLReader.h>
#include <vtkSimplePointsReader.h>
#include <vtkSmartPointer.h>
#include <vtkVRMLImporter.h>

#if _WIN32
#include <corecrt_io.h>
#endif
#include <type_traits>

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
        vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
        if constexpr (std::is_same<T, vtkGLTFImporter>::value || std::is_same<T, vtk3DSImporter>::value) {
            vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
            vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
            renderWindow->AddRenderer(renderer);
            reader->SetRenderWindow(renderWindow);
            if constexpr (std::is_same<T, vtk3DSImporter>::value) {
                reader->ComputeNormalsOn();
            }
            reader->Update();
            auto&& ta = renderWindow->GetRenderers()->GetFirstRenderer()->GetActors()->GetLastActor();
            if (!ta) {
				return nullptr;
			}
            // 深度复制
            actor->ShallowCopy(ta);
        } else {
            vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
            mapper->SetInputConnection(reader->GetOutputPort());

            actor->SetMapper(mapper);
        }
        reader->Update();

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
            { "gltf", ReadModel<vtkGLTFImporter> },
            { "3ds", ReadModel<vtk3DSImporter> },
            //         { "cml", ReadModel<vtkCMLMoleculeReader> },
            //{ "wrl", ReadModel<vtkVRMLImporter> },
            //         { "bin", ReadModel<vtkMultiBlockPLOT3DReader> },
            //{ "slc", ReadModel<vtkSLCReader> },
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
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr Read(const std::string& filename);

}; // namespace PointCloud
}; // namespace IO