#ifndef VTKWIDGET_H
#define VTKWIDGET_H

class VTKWidget : public QVTKWidget
{
    Q_OBJECT

public:
    explicit VTKWidget(QWidget *parent = 0);
    void showEvent(QShowEvent *);

private:
    pcl::visualization::PCLVisualizer *_visualizer;
    bool _initialized;
    float _rotation;
    QTimer *_timer;

private slots:
    void initInteractor();
    void updateView();
};
#endif // VTKWIDGET_H
