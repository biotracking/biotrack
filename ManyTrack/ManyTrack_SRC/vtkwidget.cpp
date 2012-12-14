#include "vtkwidget.h"

#include <QtCore/QDebug>

#include <Eigen/Geometry>

VTKWidget::VTKWidget(QWidget *parent) :
    QVTKWidget(parent),
    _visualizer(new pcl::visualization::PCLVisualizer("", false)),
    _initialized(false),
    _rotation(0.0),
    _timer(new QTimer)
{
    connect(_timer, SIGNAL(timeout()), this, SLOT(updateView()));
    this->setMinimumSize(640, 480);
}

void VTKWidget::showEvent(QShowEvent *)
{
    QTimer::singleShot(0, this, SLOT(initInteractor()));
}

void VTKWidget::initInteractor()
{
    if (!_initialized) {
        SetRenderWindow(_visualizer->getRenderWindow());
        _visualizer->setupInteractor(GetInteractor(), GetRenderWindow());
        _visualizer->addCoordinateSystem(1, 0, 0, 0);
        _visualizer->addCube(-0.5, 0.5, -0.5, 0.5, -0.5, 0.5, 1.0, 0.0, 0.0);
        _initialized = true;
        _timer->start(10);
    }
}

void VTKWidget::updateView()
{
    using namespace Eigen;
    Affine3f rotation;
    rotation = AngleAxisf(_rotation, Vector3f(1.0, 0.0, 0.0));
    _visualizer->updateShapePose("cube", rotation);
    _rotation += 0.01;
    update();
}
