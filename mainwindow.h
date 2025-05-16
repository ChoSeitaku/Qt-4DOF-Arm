#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTableWidget>
#include <QLineEdit>
#include <QPushButton>
#include <QLabel>
#include <QMenuBar>
#include <QStatusBar>
#include <QVector>
#include <QVBoxLayout>
#include <QHeaderView>
#include <QEvent>
#include <QToolTip>
#include <Qt3DExtras/Qt3DWindow>
#include <Qt3DExtras/QOrbitCameraController>
#include <Qt3DCore/QEntity>
#include <Qt3DExtras/QPhongMaterial>
#include <Qt3DExtras/QCylinderMesh>
#include <Qt3DExtras/QSphereMesh>
#include <Qt3DCore/QTransform>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void onForwardSolveClicked();
    void onInverseSolveClicked();
    void onResetClicked();
    void onInverseResultSelected(int row); // 新增
    void onZoomInClicked();  // 新增：放大视野按钮槽函数
    void onZoomOutClicked(); // 新增：缩小视野按钮槽函数

private:
    Ui::MainWindow *ui;
    QTableWidget *poseMatrixInputTable;
    QTableWidget *forwardResultTable;
    QTableWidget *inverseResultTable;
    QLineEdit *theta1Edit, *theta2Edit, *theta3Edit, *theta4Edit;
    //*theta5Edit, *theta6Edit;
    QLabel *errorLabel;
    Qt3DExtras::Qt3DWindow *view3D;   // 3D窗口容器
    Qt3DCore::QEntity *rootEntity;    // 根实体
    QWidget *container3D;             // 用于嵌入3D窗口的QWidget
    Qt3DCore::QTransform *jointTransform; // 关节的变换组件
    QVector<Qt3DCore::QTransform*> jointTransforms; // 用于存储每个关节的变换组件
    QVector<Qt3DCore::QEntity*> linkEntities; // 用于存储每个连杆的实体
    QVector<Qt3DCore::QTransform*> linkTransforms; // 用于存储每个连杆的变换组件
    Qt3DCore::QEntity *endEffector;      // 末端执行器实体
    Qt3DCore::QTransform *eeTransform;   // 末端执行器的变换组件
    Qt3DRender::QCamera *camera;
    QPushButton *zoomInButton;    // 放大视野按钮
    QPushButton *zoomOutButton;   // 缩小视野按钮

    QVector<QMatrix4x4> jointInitialTransforms;
    QVector<QMatrix4x4> linkInitialTransforms;


    // 声明正解和逆解函数
    QVector<QVector<double>> myfkine(double theta1, double theta2, double theta3, double theta4);
    QVector<QVector<double>> mymodikine(const QVector<QVector<double>> &Tbe);
    void createCoordinateAxes();
    void updateJointTransforms(const QVector<double>& angles);
    std::vector<QVector<QVector<double>>> calculateJointMatrices(double theta1, double theta2, double theta3, double theta4);

};
#endif // MAINWINDOW_H
