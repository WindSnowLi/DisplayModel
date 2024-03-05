#include "widget/DisplayModel.h"

#include <QFileDialog>
#include <QMouseEvent>
#include <QTimer>

#include <vtkAxesActor.h>
#include <vtkCamera.h>
#include <vtkCaptionActor2D.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRendererCollection.h>

#include "ui/ui_MainWindow.h"

#include "IO.h"

DisplayModel::DisplayModel(QWidget* parent)
    : QWidget(parent)
    , ui(new Ui::DisplayModel)
{
    ui->setupUi(this);

    ui->type->addItem(tr("模型"));
    ui->type->addItem(tr("点云"));

    ui->path->installEventFilter(this);

    auto renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    auto renderer = vtkSmartPointer<vtkRenderer>::New();

    renderWindow->AddRenderer(renderer);
    ui->view->setRenderWindow(renderWindow);

    ui->view->setEnableHiDPI(true);
    // 开启抗锯齿
    renderWindow->SetMultiSamples(4);
    renderer->SetUseFXAA(true);
    // 设置页面底部颜色值
    renderer->SetBackground(1.0, 1.0, 1.0);
    // 设置页面顶部颜色值
    renderer->SetBackground2(0.529, 0.8078, 0.92157);
    // 开启渐变色背景设置
    renderer->SetGradientBackground(true);

    _viewer = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer(renderer, renderWindow, "", false));

    _viewer->setupInteractor(ui->view->interactor(), renderWindow);

    // 激活同步坐标小窗
    this->_borderWidget = vtkOrientationMarkerWidget::New();
    vtkSmartPointer<vtkAxesActor> widgetAxesActor = vtkSmartPointer<vtkAxesActor>::New();
    widgetAxesActor->SetPosition(0, 0, 0);
    widgetAxesActor->SetShaftType(0);
    widgetAxesActor->SetCylinderRadius(0.02);
    // 设置大小
    widgetAxesActor->SetTotalLength(2, 2, 2);
    this->_borderWidget->SetOrientationMarker(widgetAxesActor);
    this->_borderWidget->SetInteractor(ui->view->interactor());
    this->_borderWidget->SetEnabled(true);
    this->_borderWidget->InteractiveOn();

    // 设置摄像头位置，1000，1000，1000指向0，0，0
    auto&& camera = renderer->GetActiveCamera();
    camera->SetPosition(30, 30, 30);
    // 视野Z轴向上
    camera->SetViewUp(0, 0, 1);
    camera->SetFocalPoint(0, 0, 0);

    renderer->ResetCamera();

    // 添加基础坐标系显示
    renderer->AddActor([]() {
        vtkNew<vtkAxesActor> axes;
        axes->SetTotalLength(10, 10, 10);
        axes->SetPosition(0, 0, 0);
        axes->SetShaftType(0);
        axes->SetConeResolution(100);
        axes->SetCylinderResolution(100);
        axes->SetConeRadius(0.1);
        axes->SetCylinderRadius(0.01);
        // 隐藏标签label
        axes->AxisLabelsOff();
        axes->GetXAxisCaptionActor2D()->GetProperty()->SetOpacity(0.7);
        axes->GetYAxisCaptionActor2D()->GetProperty()->SetOpacity(0.7);
        axes->GetZAxisCaptionActor2D()->GetProperty()->SetOpacity(0.7);
        return axes;
    }());

    QObject::connect(ui->loadBtn, &QPushButton::clicked, this, &DisplayModel::onLoadModel);
}

DisplayModel::~DisplayModel()
{
    delete ui;
}

bool DisplayModel::eventFilter(QObject* watched, QEvent* event)
{
    // 双击
    if (event->type() == QEvent::MouseButtonDblClick) {
        QMouseEvent* mouseEvent = static_cast<QMouseEvent*>(event);
        if (watched == ui->path && mouseEvent->button() == Qt::LeftButton) {
            this->onSelectedModel();
            return true;
        }
    }
    return QWidget::eventFilter(watched, event);
}

void DisplayModel::onSelectedModel()
{
    auto&& type = ui->type->currentText();
    auto&& filter = (type == tr("模型") ? tr("Files (*.ply *.stl *.obj *.pdb *.3ds *.gltf)") : tr("Files (*.pcd *.ply *.ifs *.obj)"));
    // ply,stl, obj, pdb
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open File"), "", filter);
    if (fileName.isEmpty()) {
        return;
    }
    ui->path->setText(fileName);
}

void DisplayModel::addModel(vtkSmartPointer<vtkActor> actor)
{
    if (actor == nullptr) {
        qDebug() << "actor is null";
        return;
    }
    ui->view->renderWindow()->GetRenderers()->GetFirstRenderer()->AddActor(actor);
    ui->view->renderWindow()->Render();
}

void DisplayModel::addPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    if (cloud == nullptr) {
        qDebug() << "cloud is null";
        return;
    }
    _viewer->addPointCloud(cloud, "cloud");
    _viewer->resetCamera();
    ui->view->renderWindow()->Render();
}

void DisplayModel::onLoadModel()
{
    QString fileName = ui->path->text();
    if (fileName.isEmpty()) {
        return;
    }
    auto&& type = ui->type->currentText();
    if (type == tr("模型")) {
        this->addModel(IO::Model::Read(fileName.toStdString()));
    } else if (type == tr("点云")) {
        this->addPointCloud(IO::PC::Read(fileName.toStdString()));
    }
}
