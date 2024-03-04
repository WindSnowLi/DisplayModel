#include "IO.h"

#include <unordered_map>

#include <vtkOBJReader.h>
#include <vtkPDBReader.h>
#include <vtkPLYReader.h>
#include <vtkPolyDataMapper.h>
#include <vtkSTLReader.h>

vtkSmartPointer<vtkActor> IO::Model::Read(const std::string& filename)
{
    // 截取文件类型
    auto&& pos = filename.find_last_of('.');
    if (pos == std::string::npos) {
        return nullptr;
    }
    auto&& type = filename.substr(pos + 1);

    static std::unordered_map<std::string, std::function<vtkSmartPointer<vtkActor>(const std::string&)>> map = {
        { "ply", ReadPLY },
        { "stl", ReadSTL },
        { "obj", ReadOBJ },
        { "pdb", ReadPDB },
    };
    if (map.find(type) == map.end()) {
        return nullptr;
    }
    return map[type](filename);
}

vtkSmartPointer<vtkActor> IO::Model::ReadPLY(const std::string& filename)
{
    vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
    reader->SetFileName(filename.c_str());

    // Visualize
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(reader->GetOutputPort());

    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    return actor;
}

vtkSmartPointer<vtkActor> IO::Model::ReadSTL(const std::string& filename)
{
    vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
    reader->SetFileName(filename.c_str());

    // Visualize
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(reader->GetOutputPort());

    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    return actor;
}

vtkSmartPointer<vtkActor> IO::Model::ReadOBJ(const std::string& filename)
{

    vtkSmartPointer<vtkOBJReader> reader = vtkSmartPointer<vtkOBJReader>::New();
    reader->SetFileName(filename.c_str());

    // Visualize
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(reader->GetOutputPort());

    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    return actor;
}

vtkSmartPointer<vtkActor> IO::Model::ReadPDB(const std::string& filename)
{
    vtkSmartPointer<vtkPDBReader> reader = vtkSmartPointer<vtkPDBReader>::New();
    reader->SetFileName(filename.c_str());

    // Visualize
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(reader->GetOutputPort());

    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    return actor;
}