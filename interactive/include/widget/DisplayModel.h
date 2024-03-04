#pragma once

#include <QWidget>

#include <vtkActor.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkSmartPointer.h>

QT_BEGIN_NAMESPACE
namespace Ui {
class DisplayModel;
}
QT_END_NAMESPACE

class DisplayModel : public QWidget {
    Q_OBJECT

public:
    DisplayModel(QWidget* parent = nullptr);
    ~DisplayModel();

    /**
     * @brief 事件过滤器监听子控件事件
     * @param watched  控件
     * @param event 事件
     * @return 是否继续传递
     */
    bool eventFilter(QObject* watched, QEvent* event) override;

protected slots:
    /**
     * @brief 响应加载模型按钮点击事件
     */
    void onLoadModel();

    /**
     * @brief 响应选择模型双击事件
     */
    void onSelectedModel();

protected:
    /**
     * @brief 添加模型
     * @param  vtkActor
     */
    void addModel(vtkSmartPointer<vtkActor>);

private:
    Ui::DisplayModel* ui;

    vtkSmartPointer<vtkOrientationMarkerWidget> _borderWidget { nullptr };
};
