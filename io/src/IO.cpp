#include "IO.h"

// OCCT
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <Bnd_Box.hxx>
#include <IGESControl_Reader.hxx>
#include <IVtkTools_ShapeDataSource.hxx>
#include <STEPCAFControl_Reader.hxx>

vtkSmartPointer<vtkActor> IO::Model::ReadSLC(const std::string& filename, double isoValue)
{
    vtkNew<vtkSLCReader> reader;
    reader->SetFileName(filename.c_str());
    reader->Update();
    vtkNew<vtkContourFilter> cFilter;
    cFilter->SetInputConnection(reader->GetOutputPort());

    cFilter->SetValue(0, isoValue);
    cFilter->Update();
    vtkNew<vtkOutlineFilter> outliner;
    outliner->SetInputConnection(reader->GetOutputPort());
    outliner->Update();
    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputConnection(cFilter->GetOutputPort());
    mapper->SetScalarVisibility(0);

    vtkNew<vtkActor> actor;
    actor->SetMapper(mapper);
    return actor;
}

vtkSmartPointer<vtkActor> IO::Model::ReadIGES(const std::string& filename)
{
    IGESControl_Reader reader;
    reader.ReadFile(filename.c_str());
    reader.TransferRoots();
    TopoDS_Shape shape = reader.OneShape();
    vtkNew<vtkPolyData> polydata;
    vtkNew<vtkPolyDataMapper> mapper;
    vtkNew<vtkActor> actor;
    vtkNew<IVtkTools_ShapeDataSource> shapeSource;
    shapeSource->SetShape(new IVtkOCC_Shape(shape));
    shapeSource->Update();
    mapper->SetInputConnection(shapeSource->GetOutputPort());
    actor->SetMapper(mapper);
    return actor;
}

vtkSmartPointer<vtkActor> IO::Model::ReadSTEP(const std::string& filename)
{
    STEPControl_Reader readerSTEP;
    IFSelect_ReturnStatus statSTEP = readerSTEP.ReadFile(filename.c_str());
    IFSelect_PrintCount modeSTEP = IFSelect_ListByItem;
    readerSTEP.PrintCheckLoad(Standard_False, modeSTEP);
    readerSTEP.TransferRoots();
    TopoDS_Shape shape = readerSTEP.OneShape();

    vtkNew<vtkPolyDataMapper> mapper;
    vtkNew<vtkActor> actor;
    vtkNew<IVtkTools_ShapeDataSource> shapeSource;

    shapeSource->SetShape(new IVtkOCC_Shape(shape));
    shapeSource->Update();
    mapper->SetInputConnection(shapeSource->GetOutputPort());
    actor->SetMapper(mapper);
    return actor;
}

vtkSmartPointer<vtkActor> IO::Model::Read(const std::string& filename)
{
    // 截取文件类型
    auto&& pos = filename.find_last_of('.');
    if (pos == std::string::npos) {
        return nullptr;
    }
    auto&& type = filename.substr(pos + 1);

    // 转为小写
    std::transform(type.begin(), type.end(), type.begin(), ::tolower);

    static std::unordered_map<std::string, std::function<vtkSmartPointer<vtkActor>(const std::string&)>> map = {
        { "ply", ReadModel<vtkPLYReader> },
        { "stl", ReadModel<vtkSTLReader> },
        { "obj", ReadModel<vtkOBJReader> },
        { "pdb", ReadModel<vtkPDBReader> },
        { "igs", ReadIGES },
        { "step", ReadSTEP },
        //{ "gltf", ReadModel<vtkGLTFImporter> },
        //{ "3ds", ReadModel<vtk3DSImporter> },
        //         { "cml", ReadModel<vtkCMLMoleculeReader> },
        //{ "wrl", ReadModel<vtkVRMLImporter> },
        //         { "bin", ReadModel<vtkMultiBlockPLOT3DReader> },
        {
            "slc",
            [](const std::string& s) {
                return ReadSLC(s);
            } }
    };
    if (map.find(type) == map.end()) {
        return nullptr;
    }
    return map[type](filename);
}

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
