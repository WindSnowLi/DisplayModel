#include "IO.h"

#include <vtk3DSImporter.h>
#include <vtkAppendPolyData.h>
#include <vtkCMLMoleculeReader.h>
#include <vtkCleanPolyData.h>
#include <vtkContourFilter.h>
#include <vtkExodusIIReader.h>
#include <vtkExtractVOI.h>
#include <vtkGLTFImporter.h>
#include <vtkMultiBlockPLOT3DReader.h>
#include <vtkOBJImporter.h>
#include <vtkOBJReader.h>
#include <vtkOutlineFilter.h>
#include <vtkPDBReader.h>
#include <vtkPLYReader.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRendererCollection.h>
#include <vtkSLCReader.h>
#include <vtkSTLReader.h>
#include <vtkSimplePointsReader.h>
#include <vtkVRMLImporter.h>

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

#include <unordered_map>

#if _WIN32
#include <corecrt_io.h>
#endif

#include <pcl/io/auto_io.h>

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

vtkSmartPointer<vtkActor> IO::Model::ReadIGESOnMeshes(const std::string& filename)
{
    STEPControl_Reader readerSTEP;
    IFSelect_ReturnStatus statSTEP = readerSTEP.ReadFile(filename.c_str());
    IFSelect_PrintCount modeSTEP = IFSelect_ListByItem;
    readerSTEP.PrintCheckLoad(Standard_False, modeSTEP);
    readerSTEP.TransferRoots();
    TopoDS_Shape TopoDS_ShapeSTEP = readerSTEP.OneShape();

    // VTKpointOnMeshes
    vtkNew<vtkPoints> VTKpointsOnMeshes;
    vtkNew<vtkCellArray> verticesForVTKpointsOnMeshes;
    vtkIdType pid[1];

    // vtkPolyData creation
    vtkNew<vtkPolyData> PolyDataVTKPointsOnMeshes;

    // vtkPolyData initialization
    PolyDataVTKPointsOnMeshes->SetPoints(VTKpointsOnMeshes);
    PolyDataVTKPointsOnMeshes->SetVerts(verticesForVTKpointsOnMeshes);

    // VTKpoint IGES, STEP TopoDS_ShapeBOX CALC
    Bnd_Box aabb;
    // AABB ALGO Per TopoDS_Shape mesh  , Please choose the source you like to run and put it in next line
    BRepBndLib::Add(TopoDS_ShapeSTEP, aabb, true); // TopoDS_ShapeIGES  //TopoDS_ShapeSTEP //TopoDS_ShapeCONE
    // VTKpointIGES CALC per TopoDS_Shape mesh
    int density = 20;
    gp_XYZ Pmin = aabb.CornerMin().XYZ();
    gp_XYZ Pmax = aabb.CornerMax().XYZ();
    gp_XYZ D = Pmax - Pmin;
    double dim[3] = { D.X(), D.Y(), D.Z() };
    double mind = Min(dim[0], Min(dim[1], dim[2]));
    const double d = mind / density;
    int nslice[3] = {
        int(Round(dim[0] / d)) + 1,
        int(Round(dim[1] / d)) + 1,
        int(Round(dim[2] / d)) + 1
    };
    for (int i = 0; i < nslice[0]; ++i)
        for (int j = 0; j < nslice[1]; ++j)
            for (int k = 0; k < nslice[2]; ++k) {
                gp_XYZ p = Pmin
                    + gp_XYZ(d * i, 0, 0)
                    + gp_XYZ(0, d * j, 0)
                    + gp_XYZ(0, 0, d * k);
                pid[0] = VTKpointsOnMeshes->InsertNextPoint(p.X(), p.Y(), p.Z());
                verticesForVTKpointsOnMeshes->InsertNextCell(1, pid);
            };

    // Append the meshes
    vtkNew<vtkAppendPolyData> appendFilter;
    appendFilter->AddInputData(PolyDataVTKPointsOnMeshes);

    // Remove any duplicate points.
    vtkNew<vtkCleanPolyData> cleanFilterForMapperVTKShapes;
    cleanFilterForMapperVTKShapes->SetInputConnection(appendFilter->GetOutputPort());
    cleanFilterForMapperVTKShapes->Update();

    vtkNew<vtkPolyDataMapper> mapperVTKShapes;
    mapperVTKShapes->SetInputConnection(cleanFilterForMapperVTKShapes->GetOutputPort());

    vtkNew<vtkActor> actorVTKShapes;
    actorVTKShapes->SetMapper(mapperVTKShapes);
    return actorVTKShapes;
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
