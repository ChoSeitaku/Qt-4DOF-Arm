#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <cmath>
#include <QMessageBox>
#include <QHeaderView>
#include <QTextEdit>
#include <Qt3DExtras/Qt3DWindow>
#include <Qt3DExtras/QOrbitCameraController>
#include <Qt3DCore/QEntity>
#include <Qt3DExtras/QPhongMaterial>
#include <Qt3DExtras/QCylinderMesh>
#include <Qt3DExtras/QSphereMesh>
#include <Qt3DCore/QTransform>
#include <Qt3DRender/QCamera>
#include <Qt3DRender/QRenderSettings>
#include <Qt3DRender/QFrameGraphNode>
#include <Qt3DRender/QClearBuffers>
#include <QtGui/QColor>
#include <Qt3DExtras/QForwardRenderer>
#include <QPointLight>
#include <Qt3DExtras/QDiffuseSpecularMaterial>
#include <QTimer>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // 创建3D窗口
    view3D = new Qt3DExtras::Qt3DWindow();
    container3D = QWidget::createWindowContainer(view3D, this); // 将3D窗口嵌入到QWidget
    container3D->setMinimumSize(QSize(400, 300)); // 设置最小尺寸

    // 创建根实体
    rootEntity = new Qt3DCore::QEntity();

    // 设置相机
    camera = view3D->camera();
    // 调整视点位置，使相机远离机械臂，扩大展示范围
    camera->setPosition(QVector3D(5, 5, 5));
    camera->setViewCenter(QVector3D(0, 0, 0));
    camera->setUpVector(QVector3D(0, 1, 0));
    camera->setProjectionType(Qt3DRender::QCameraLens::PerspectiveProjection);
    // 增大视场角，从45.0f调整为60.0f
    camera->setFieldOfView(60.0f);
    // 调整近裁剪面，适当缩小，从0.1f调整为0.01f
    camera->setNearPlane(0.01f);
    // 调整远裁剪面，适当增大，从1000.0f调整为2000.0f
    camera->setFarPlane(2000.0f);



    // 设置光照
    Qt3DCore::QEntity *lightEntity = new Qt3DCore::QEntity(rootEntity);
    Qt3DRender::QPointLight *light = new Qt3DRender::QPointLight(lightEntity);
    light->setColor(Qt::white);
    light->setIntensity(1.0f);
    lightEntity->addComponent(light);

    // 设置光源位置
    Qt3DCore::QTransform *lightTransform = new Qt3DCore::QTransform(lightEntity);
    lightTransform->setTranslation(QVector3D(5, 5, 5)); // 调整光源位置
    lightEntity->addComponent(lightTransform);

    // 创建坐标系辅助线
    createCoordinateAxes();

    // 创建末端执行器（红色球体）
    endEffector = new Qt3DCore::QEntity(rootEntity);
    Qt3DExtras::QSphereMesh *eeMesh = new Qt3DExtras::QSphereMesh();
    eeMesh->setRadius(0.08f); // 半径8厘米
    Qt3DExtras::QPhongMaterial *eeMaterial = new Qt3DExtras::QPhongMaterial();
    eeMaterial->setDiffuse(Qt::red); // 红色材质
    eeTransform = new Qt3DCore::QTransform();
    endEffector->addComponent(eeMesh);
    endEffector->addComponent(eeMaterial);
    endEffector->addComponent(eeTransform);



    // // 1. 创建关节实体
    // Qt3DCore::QEntity *joint1 = new Qt3DCore::QEntity(rootEntity);

    // // 2. 添加圆柱体网格（表示关节）
    // Qt3DExtras::QCylinderMesh *jointMesh = new Qt3DExtras::QCylinderMesh(joint1);
    // jointMesh->setRadius(0.1f);    // 半径
    // jointMesh->setLength(0.5f);    // 长度
    // jointMesh->setRings(10);       // 细分段数（提高渲染质量）

    // // 3. 添加材质（黄色）
    // Qt3DExtras::QDiffuseSpecularMaterial *jointMaterial = new Qt3DExtras::QDiffuseSpecularMaterial(joint1);
    // jointMaterial->setDiffuse(QColor(255, 255, 0)); // 黄色
    // jointMaterial->setSpecular(QColor(0, 0, 0, 0)); // 禁用高光

    // // 4. 添加变换组件（控制位置和旋转）
    // jointTransform = new Qt3DCore::QTransform(joint1);
    // jointTransform->setTranslation(QVector3D(0, 0.25f, 0)); // 居中（长度的一半）

    // // 5. 将组件附加到实体
    // joint1->addComponent(jointMesh);
    // joint1->addComponent(jointMaterial);
    // joint1->addComponent(jointTransform);
    // 关节参数配置（单位：米）
    // 关节位置参数
    const QVector<QVector3D> jointPositions = {
        QVector3D(0.0, 0.0, 0.0),     // 关节1位置
        QVector3D(0.0, 0.2, 0.0),     // 关节2位置
        QVector3D(0.0, 0.4, 0.0),     // 关节3位置
        QVector3D(0.0, 0.6, 0.0),     // 关节4位置
        //QVector3D(0.0, 0.8, 0.0),     // 关节5位置
        //QVector3D(0.0, 1.0, 0.0)      // 关节6位置
    };

    // MDH参数格式：{theta_i, d_i, a_i, alpha_i}
    const QVector<QVector<double>> MDH = {
        {0, 0, 0, 0},               // 关节1: alpha=0（绕X轴旋转0°）
        {0, 0, 0.325, -M_PI/2},     // 关节2: alpha=-90°（绕X轴旋转-90°）
        {0, 0, 1.150, 0},           // 关节3: alpha=0
        {0, 1.225, 0.300, -M_PI/2}, // 关节4: alpha=-90°
        //{0, 0, 0, M_PI/2},          // 关节5: alpha=90°
        //{0, 0, 0, -M_PI/2}          // 关节6: alpha=-90°
    };

    // 将根实体绑定到窗口
    view3D->setRootEntity(rootEntity);

    for(int i = 0; i < 4; i++){
        Qt3DCore::QEntity *joint = new Qt3DCore::QEntity(rootEntity);
        // 修改为球体网格
        Qt3DExtras::QSphereMesh *mesh = new Qt3DExtras::QSphereMesh();
        mesh->setRadius(0.05); // 设置球体半径

        // Qt3DCore::QTransform *transform = new Qt3DCore::QTransform();
        // transform->setTranslation(jointParams[i]);

        Qt3DExtras::QPhongMaterial *material = new Qt3DExtras::QPhongMaterial();
        material->setDiffuse(QColor(255, 255, 0)); // 黄色

        // 变换组件（设置位置和初始旋转轴）
        Qt3DCore::QTransform *transform = new Qt3DCore::QTransform(joint);
        transform->setTranslation(jointPositions[i]); // 设置位置
        // transform->setRotationX(qRadiansToDegrees(MDH[i][3])); // 设置绕X轴的初始旋转（alpha_i）
        transform->setRotationX(0); // 初始无旋转

        joint->addComponent(mesh);
        joint->addComponent(material);
        joint->addComponent(transform);
        jointTransforms.append(transform);

        // 创建连杆
        if (i > 0) {
            Qt3DCore::QEntity *link = new Qt3DCore::QEntity(rootEntity);
            Qt3DExtras::QCylinderMesh *linkMesh = new Qt3DExtras::QCylinderMesh();
            linkMesh->setRadius(0.03);

            // 计算连杆的长度和方向
            QVector3D start = jointPositions[i - 1];
            QVector3D end = jointPositions[i];
            QVector3D direction = end - start;
            float length = direction.length();
            linkMesh->setLength(length);

            // 计算连杆的旋转
            QVector3D up = QVector3D(0, 1, 0);
            QVector3D axis = QVector3D::crossProduct(up, direction.normalized());
            float angle = qRadiansToDegrees(qAcos(QVector3D::dotProduct(up, direction.normalized())));

            Qt3DCore::QTransform *linkTransform = new Qt3DCore::QTransform();
            linkTransform->setTranslation(start + direction / 2);
            linkTransform->setRotation(QQuaternion::fromAxisAndAngle(axis, angle));

            Qt3DExtras::QPhongMaterial *linkMaterial = new Qt3DExtras::QPhongMaterial();
            linkMaterial->setDiffuse(QColor(128, 128, 128)); // 灰色
            link->addComponent(linkMesh);
            link->addComponent(linkMaterial);
            link->addComponent(linkTransform);

            linkEntities.append(link);
            linkTransforms.append(linkTransform);
        }
    }

    // 记录关节和连杆的初始变换
    for (int i = 0; i < jointTransforms.size(); ++i) {
        jointInitialTransforms.append(jointTransforms[i]->matrix());
    }
    for (int i = 0; i < linkTransforms.size(); ++i) {
        linkInitialTransforms.append(linkTransforms[i]->matrix());
    }

    this->setWindowTitle("4自由度磨抛机器人控制界面");

    // 创建输入输出表格
    poseMatrixInputTable = new QTableWidget(4, 4, this);
    forwardResultTable = new QTableWidget(4, 4, this);
    inverseResultTable = new QTableWidget(8, 6, this);

    // 创建关节角度输入框
    theta1Edit = new QLineEdit(this);
    theta2Edit = new QLineEdit(this);
    theta3Edit = new QLineEdit(this);
    theta4Edit = new QLineEdit(this);
    //theta5Edit = new QLineEdit(this);
    //theta6Edit = new QLineEdit(this);

    // 创建按钮
    QPushButton *forwardSolveButton = new QPushButton("正解计算", this);
    QPushButton *inverseSolveButton = new QPushButton("逆解计算", this);
    QPushButton *resetButton = new QPushButton("复位", this);

    // 创建错误提示标签
    errorLabel = new QLabel(this);

    // 设置表格表头
    QStringList horizontalHeaders = {"R11", "R12", "R13", "T1"};
    QStringList verticalHeaders = {"R21", "R22", "R23", "T2"};
    poseMatrixInputTable->setHorizontalHeaderLabels(horizontalHeaders);
    poseMatrixInputTable->setVerticalHeaderLabels(verticalHeaders);
    forwardResultTable->setHorizontalHeaderLabels(horizontalHeaders);
    forwardResultTable->setVerticalHeaderLabels(verticalHeaders);

    horizontalHeaders << "关节1" << "关节2" << "关节3" << "关节4" << "关节5" << "关节6";
    inverseResultTable->setHorizontalHeaderLabels(horizontalHeaders);

    // 布局管理
    QVBoxLayout *inputLayout = new QVBoxLayout;
    inputLayout->addWidget(new QLabel("关节角度输入："));
    inputLayout->addWidget(theta1Edit);
    inputLayout->addWidget(theta2Edit);
    inputLayout->addWidget(theta3Edit);
    inputLayout->addWidget(theta4Edit);
    //inputLayout->addWidget(theta5Edit);
    //inputLayout->addWidget(theta6Edit);
    inputLayout->addWidget(new QLabel("末端执行器位姿矩阵输入："));
    inputLayout->addWidget(poseMatrixInputTable);
    inputLayout->addWidget(forwardSolveButton);
    inputLayout->addWidget(inverseSolveButton);
    inputLayout->addWidget(resetButton);
    inputLayout->addWidget(errorLabel);

    QVBoxLayout *outputLayout = new QVBoxLayout;
    outputLayout->addWidget(new QLabel("正解结果："));
    outputLayout->addWidget(forwardResultTable);
    outputLayout->addWidget(new QLabel("逆解结果："));
    outputLayout->addWidget(inverseResultTable);

    QHBoxLayout *mainLayout = new QHBoxLayout;
    mainLayout->addLayout(inputLayout);
    mainLayout->addLayout(outputLayout);

    QWidget *centralWidget = new QWidget(this);
    centralWidget->setLayout(mainLayout);
    setCentralWidget(centralWidget);


    // 连接信号槽
    connect(forwardSolveButton, &QPushButton::clicked, this, &MainWindow::onForwardSolveClicked);
    connect(inverseSolveButton, &QPushButton::clicked, this, &MainWindow::onInverseSolveClicked);
    connect(resetButton, &QPushButton::clicked, this, &MainWindow::onResetClicked);
    connect(inverseResultTable, &QTableWidget::cellClicked, this, &MainWindow::onInverseResultSelected);

    // 创建菜单栏和状态栏
    QMenu *fileMenu = menuBar()->addMenu("文件");
    QAction *exitAction = new QAction("退出", this);
    connect(exitAction, &QAction::triggered, qApp, &QApplication::quit);
    fileMenu->addAction(exitAction);

    statusBar()->showMessage("准备就绪");

    // 为关节角度输入框添加工具提示，提示用户输入有效的数字
    theta1Edit->setToolTip("请输入关节1的角度值（单位：弧度）");
    theta2Edit->setToolTip("请输入关节2的角度值（单位：弧度）");
    theta3Edit->setToolTip("请输入关节3的角度值（单位：弧度）");
    theta4Edit->setToolTip("请输入关节4的角度值（单位：弧度）");
    //theta5Edit->setToolTip("请输入关节5的角度值（单位：弧度）");
    //theta6Edit->setToolTip("请输入关节6的角度值（单位：弧度）");

    // 为末端执行器位姿矩阵输入表格添加工具提示，提示用户输入有效的矩阵元素
    poseMatrixInputTable->setToolTip("请输入末端执行器位姿矩阵的元素，每个单元格输入一个数字");

    // 为正解结果输出表格添加工具提示，提示用户查看正解计算结果
    forwardResultTable->setToolTip("正解计算结果将显示在此表格中，格式为4x4的位姿矩阵");

    // 为逆解结果输出表格添加工具提示，提示用户查看逆解计算结果
    inverseResultTable->setToolTip("逆解计算结果将显示在此表格中，每行表示一组关节角度解");

    // 为正解计算按钮添加工具提示
    forwardSolveButton->setToolTip("点击此按钮进行正解计算，根据输入的关节角度计算末端执行器位姿矩阵");

    // 为逆解计算按钮添加工具提示
    inverseSolveButton->setToolTip("点击此按钮进行逆解计算，根据输入的末端执行器位姿矩阵计算关节角度");

    // 为复位按钮添加工具提示
    resetButton->setToolTip("点击此按钮将所有输入框和表格清空，重置错误提示信息");

    // 创建展开/折叠按钮
    QPushButton *toggleButton = new QPushButton("矩阵和机械臂说明", this);

    // 创建一个对话框用于显示矩阵描述
    QDialog *descriptionDialog = new QDialog(this);
    descriptionDialog->setWindowTitle("矩阵和机械臂说明");
    descriptionDialog->setMinimumSize(400, 300); // 设置对话框的最小尺寸

    // 创建文本编辑框用于显示矩阵描述
    QTextEdit *descriptionTextEdit = new QTextEdit(descriptionDialog);
    descriptionTextEdit->setReadOnly(true);

    // 矩阵描述内容
    QString matrixDescription = "矩阵各元素含义说明\n\n"
                                "旋转矩阵部分\n"
                                "- R11：旋转矩阵第一行第一列元素，代表末端执行器在基础坐标系下X轴旋转分量。\n"
                                "- R12：旋转矩阵第一行第二列元素，代表末端执行器在基础坐标系下Y轴旋转分量。\n"
                                "- R13：旋转矩阵第一行第三列元素，代表末端执行器在基础坐标系下Z轴旋转分量。\n"
                                "- R21：旋转矩阵第二行第一列元素，代表末端执行器在基础坐标系下X轴旋转分量。\n"
                                "- R22：旋转矩阵第二行第二列元素，代表末端执行器在基础坐标系下Y轴旋转分量。\n"
                                "- R23：旋转矩阵第二行第三列元素，代表末端执行器在基础坐标系下Z轴旋转分量。\n\n"
                                "平移向量部分\n"
                                "- T1：平移向量第一个元素，代表末端执行器在基础坐标系下X轴的平移分量。\n"
                                "- T2：平移向量第二个元素，代表末端执行器在基础坐标系下Y轴的平移分量。\n\n"
                                "机械臂 3D 显示相关说明\n"
                                "- 关节表示：在 3D 显示中，每个关节用黄色球体表示，其位置根据机械臂的运动学模型和关节角度确定。当输入不同的关节角度进行正解计算时，关节的旋转会根据输入的角度值进行更新，从而改变机械臂的姿态。\n"
                                "- 连杆表示：连杆用灰色圆柱体表示，连接相邻的两个关节。连杆的长度和方向会根据关节的位置动态计算和更新。当关节位置改变时，连杆会自动调整其长度、方向和旋转，以正确连接两个关节。\n"
                                "- 末端执行器表示：末端执行器用红色球体表示，其位姿由旋转矩阵和平移向量共同确定。在进行正解计算后，末端执行器会根据计算得到的位姿矩阵更新其位置和姿态，在 3D 场景中同步显示。\n"
                                "- 坐标系辅助线：为了便于观察和理解机械臂的运动，3D 场景中添加了坐标系辅助线。X 轴为红色圆柱体，Y 轴为绿色圆柱体，Z 轴为蓝色圆柱体，帮助确定机械臂在空间中的位置和方向。\n";
    descriptionTextEdit->setPlainText(matrixDescription);

    // 创建关闭按钮
    QPushButton *closeButton = new QPushButton("关闭", descriptionDialog);
    connect(closeButton, &QPushButton::clicked, descriptionDialog, &QDialog::close);

    // 设置对话框的布局
    QVBoxLayout *dialogLayout = new QVBoxLayout(descriptionDialog);
    dialogLayout->addWidget(descriptionTextEdit);
    dialogLayout->addWidget(closeButton);

    // 连接按钮点击信号到槽函数
    connect(toggleButton, &QPushButton::clicked, [=]() {
        if (descriptionDialog->isHidden()) {
            descriptionDialog->show();
            toggleButton->setText("折叠矩阵描述");
        } else {
            descriptionDialog->close();
            toggleButton->setText("展开矩阵描述");
        }
    });

    // 创建一个垂直布局用于放置按钮
   // QVBoxLayout *mainLayout = new QVBoxLayout;
    mainLayout->addWidget(toggleButton);
    mainLayout->addWidget(container3D); // 假设mainLayout是QHBoxLayout

    // 创建一个中心部件并设置布局
   // QWidget *centralWidget = new QWidget(this);
    centralWidget->setLayout(mainLayout);
    setCentralWidget(centralWidget);

    // 创建放大和缩小按钮
    QPushButton *zoomInButton = new QPushButton("放大", this);
    QPushButton *zoomOutButton = new QPushButton("缩小", this);

    // 将按钮添加到布局中
    // inputLayout = new QVBoxLayout;
    // 其他已有代码...
    inputLayout->addWidget(zoomInButton);
    inputLayout->addWidget(zoomOutButton);

    // 连接信号槽
    connect(zoomInButton, &QPushButton::clicked, this, &MainWindow::onZoomInClicked);
    connect(zoomOutButton, &QPushButton::clicked, this, &MainWindow::onZoomOutClicked);
}

MainWindow::~MainWindow()
{
    delete ui;
}

QVector<QVector<double>> multiplyMatrix(const QVector<QVector<double>>& m1, const QVector<QVector<double>>& m2);

QVector<QVector<double>> MainWindow::myfkine(double theta1, double theta2, double theta3, double theta4)
{
    // 根据原MATLAB代码中的myfkine函数逻辑实现
    QVector<QVector<double>> MDH = {
        {theta1, 0, 0, 0},
        {theta2, 0, 0.325, -M_PI/2},
        {theta3, 0, 1.150, 0},
        {theta4, 1.225, 0.300, -M_PI/2}
        //{theta5, 0, 0, M_PI/2},
        //{theta6, 0, 0, -M_PI/2}
    };

    QVector<QVector<double>> T01 = {
        {cos(MDH[0][0]), -sin(MDH[0][0]), 0, MDH[0][2]},
        {cos(MDH[0][3])*sin(MDH[0][0]), cos(MDH[0][3])*cos(MDH[0][0]), -sin(MDH[0][3]), -MDH[0][1]*sin(MDH[0][3])},
        {sin(MDH[0][3])*sin(MDH[0][0]), sin(MDH[0][3])*cos(MDH[0][0]), cos(MDH[0][3]), MDH[0][1]*cos(MDH[0][3])},
        {0, 0, 0, 1}
    };

    QVector<QVector<double>> T12 = {
        {cos(MDH[1][0]), -sin(MDH[1][0]), 0, MDH[1][2]},
        {cos(MDH[1][3])*sin(MDH[1][0]), cos(MDH[1][3])*cos(MDH[1][0]), -sin(MDH[1][3]), -MDH[1][1]*sin(MDH[1][3])},
        {sin(MDH[1][3])*sin(MDH[1][0]), sin(MDH[1][3])*cos(MDH[1][0]), cos(MDH[1][3]), MDH[1][1]*cos(MDH[1][3])},
        {0, 0, 0, 1}
    };

    QVector<QVector<double>> T23 = {
        {cos(MDH[2][0]), -sin(MDH[2][0]), 0, MDH[2][2]},
        {cos(MDH[2][3])*sin(MDH[2][0]), cos(MDH[2][3])*cos(MDH[2][0]), -sin(MDH[2][3]), -MDH[2][1]*sin(MDH[2][3])},
        {sin(MDH[2][3])*sin(MDH[2][0]), sin(MDH[2][3])*cos(MDH[2][0]), cos(MDH[2][3]), MDH[2][1]*cos(MDH[2][3])},
        {0, 0, 0, 1}
    };

    QVector<QVector<double>> T34 = {
        {cos(MDH[3][0]), -sin(MDH[3][0]), 0, MDH[3][2]},
        {cos(MDH[3][3])*sin(MDH[3][0]), cos(MDH[3][3])*cos(MDH[3][0]), -sin(MDH[3][3]), -MDH[3][1]*sin(MDH[3][3])},
        {sin(MDH[3][3])*sin(MDH[3][0]), sin(MDH[3][3])*cos(MDH[3][0]), cos(MDH[3][3]), MDH[3][1]*cos(MDH[3][3])},
        {0, 0, 0, 1}
    };

    // QVector<QVector<double>> T45 = {
    //     {cos(MDH[4][0]), -sin(MDH[4][0]), 0, MDH[4][2]},
    //     {cos(MDH[4][3])*sin(MDH[4][0]), cos(MDH[4][3])*cos(MDH[4][0]), -sin(MDH[4][3]), -MDH[4][1]*sin(MDH[4][3])},
    //     {sin(MDH[4][3])*sin(MDH[4][0]), sin(MDH[4][3])*cos(MDH[4][0]), cos(MDH[4][3]), MDH[4][1]*cos(MDH[4][3])},
    //     {0, 0, 0, 1}
    // };

    // QVector<QVector<double>> T56 = {
    //     {cos(MDH[5][0]), -sin(MDH[5][0]), 0, MDH[5][2]},
    //     {cos(MDH[5][3])*sin(MDH[5][0]), cos(MDH[5][3])*cos(MDH[5][0]), -sin(MDH[5][3]), -MDH[5][1]*sin(MDH[5][3])},
    //     {sin(MDH[5][3])*sin(MDH[5][0]), sin(MDH[5][3])*cos(MDH[5][0]), cos(MDH[5][3]), MDH[5][1]*cos(MDH[5][3])},
    //     {0, 0, 0, 1}
    // };

    // 矩阵相乘计算T04
    QVector<QVector<double>> T04 = T01;
    T04 = multiplyMatrix(T04, T12);
    T04 = multiplyMatrix(T04, T23);
    T04 = multiplyMatrix(T04, T34);
    //T06 = multiplyMatrix(T06, T45);
    //T06 = multiplyMatrix(T06, T56);

    return T04;
}

// 定义multiplyMatrix函数
QVector<QVector<double>> multiplyMatrix(const QVector<QVector<double>>& m1, const QVector<QVector<double>>& m2) {
    QVector<QVector<double>> result(4, QVector<double>(4, 0));
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            for (int k = 0; k < 4; ++k) {
                result[i][j] += m1[i][k] * m2[k][j];
            }
        }
    }
    return result;
}

QVector<QVector<double>> MainWindow::mymodikine(const QVector<QVector<double>> &Tbe)
{
    // 根据原MATLAB代码中的mymodikine函数逻辑实现
    const double deg = M_PI / 180;
    QVector<QVector<double>> MDH = {
        {0, 0, 0, 0, -180 * deg, 180 * deg},
        {0, 0, 0.325, -M_PI / 2, -60 * deg, 76 * deg},
        {0, 0, 1.150, 0, -147 * deg, 90 * deg},
        {0, 1.225, 0.300, -M_PI / 2, -210 * deg, 210 * deg}
        // {0, 0, 0, M_PI / 2, -130 * deg, 130 * deg},
        // {0, 0, 0, -M_PI / 2, -210 * deg, 210 * deg}
    };

    // 提取Tbe中的元素
    double nx = Tbe[0][0], ny = Tbe[1][0], nz = Tbe[2][0];
    double ox = Tbe[0][1], oy = Tbe[1][1], oz = Tbe[2][1];
    double ax = Tbe[0][2], ay = Tbe[1][2], az = Tbe[2][2];
    double px = Tbe[0][3], py = Tbe[1][3], pz = Tbe[2][3];

    double d4 = MDH[3][1], d2 = 0, d3 = 0;
    double a1 = MDH[1][2], a2 = MDH[2][2], a3 = MDH[3][2];
    double f1 = -M_PI / 2, f3 = -M_PI / 2, f4 = M_PI / 2, f5 = -M_PI / 2;

    QVector<QVector<double>> ikine_t(8, QVector<double>(4));

    // 计算t1
    double t11 = -atan2(-py, px)+atan2((d2 - d3) / sin(f1), sqrt(pow(px * sin(f1), 2)+pow(py * sin(f1), 2)-pow(d2 - d3, 2)));
    double t12 = -atan2(-py, px)+atan2((d2 - d3) / sin(f1), -sqrt(pow(px * sin(f1), 2)+pow(py * sin(f1), 2)-pow(d2 - d3, 2)));

    // 检查t11和t12是否在有效范围内
    t11 = qBound(MDH[0][4], t11, MDH[0][5]);
    t12 = qBound(MDH[0][4], t12, MDH[0][5]);

    // 计算t3
    double m3_1 = pz * sin(f1);
    double n3_1 = a1 - px * cos(t11)-py * sin(t11);
    double m3_2 = pz * sin(f1);
    double n3_2 = a1 - px * cos(t12)-py * sin(t12);

    double t31 = -atan2(a2 * a3 / sin(f3), a2 * d4)+atan2((pow(m3_1, 2)+pow(n3_1, 2)-a2 * a2 - a3 * a3 - d4 * d4) / sin(f3),
                                                         sqrt(pow(2 * a2 * d4 * sin(f3), 2)+pow(2 * a2 * a3, 2)-pow(pow(m3_1, 2)+pow(n3_1, 2)-a2 * a2 - a3 * a3 - d4 * d4, 2)));
    double t32 = -atan2(a2 * a3 / sin(f3), a2 * d4)+atan2((pow(m3_1, 2)+pow(n3_1, 2)-a2 * a2 - a3 * a3 - d4 * d4) / sin(f3),
                                                         -sqrt(pow(2 * a2 * d4 * sin(f3), 2)+pow(2 * a2 * a3, 2)-pow(pow(m3_1, 2)+pow(n3_1, 2)-a2 * a2 - a3 * a3 - d4 * d4, 2)));
    double t33 = -atan2(a2 * a3 / sin(f3), a2 * d4)+atan2((pow(m3_2, 2)+pow(n3_2, 2)-a2 * a2 - a3 * a3 - d4 * d4) / sin(f3),
                                                         sqrt(pow(2 * a2 * d4 * sin(f3), 2)+pow(2 * a2 * a3, 2)-pow(pow(m3_2, 2)+pow(n3_2, 2)-a2 * a2 - a3 * a3 - d4 * d4, 2)));
    double t34 = -atan2(a2 * a3 / sin(f3), a2 * d4)+atan2((pow(m3_2, 2)+pow(n3_2, 2)-a2 * a2 - a3 * a3 - d4 * d4) / sin(f3),
                                                         -sqrt(pow(2 * a2 * d4 * sin(f3), 2)+pow(2 * a2 * a3, 2)-pow(pow(m3_2, 2)+pow(n3_2, 2)-a2 * a2 - a3 * a3 - d4 * d4, 2)));

    // 检查t31 - t34是否在有效范围内
    t31 = qBound(MDH[2][4], t31, MDH[2][5]);
    t32 = qBound(MDH[2][4], t32, MDH[2][5]);
    t33 = qBound(MDH[2][4], t33, MDH[2][5]);
    t34 = qBound(MDH[2][4], t34, MDH[2][5]);

    // 计算t2
    double m2_1 = a2 + a3 * cos(t31)+d4 * sin(f3) * sin(t31);
    double n2_1 = a3 * sin(t31)-d4 * sin(f3) * cos(t31);
    double m2_2 = a2 + a3 * cos(t32)+d4 * sin(f3) * sin(t32);
    double n2_2 = a3 * sin(t32)-d4 * sin(f3) * cos(t32);
    double m2_3 = a2 + a3 * cos(t33)+d4 * sin(f3) * sin(t33);
    double n2_3 = a3 * sin(t33)-d4 * sin(f3) * cos(t33);
    double m2_4 = a2 + a3 * cos(t34)+d4 * sin(f3) * sin(t34);
    double n2_4 = a3 * sin(t34)-d4 * sin(f3) * cos(t34);

    double t21 = atan2(m3_1 * m2_1 + n2_1 * n3_1, m3_1 * n2_1 - m2_1 * n3_1);
    double t22 = atan2(m3_1 * m2_2 + n2_2 * n3_1, m3_1 * n2_2 - m2_2 * n3_1);
    double t23 = atan2(m3_2 * m2_3 + n2_3 * n3_2, m3_2 * n2_3 - m2_3 * n3_2);
    double t24 = atan2(m3_2 * m2_4 + n2_4 * n3_2, m3_2 * n2_4 - m2_4 * n3_2);

    // 检查t21 - t24是否在有效范围内
    t21 = qBound(MDH[1][4], t21, MDH[1][5]);
    t22 = qBound(MDH[1][4], t22, MDH[1][5]);
    t23 = qBound(MDH[1][4], t23, MDH[1][5]);
    t24 = qBound(MDH[1][4], t24, MDH[1][5]);

    // 计算t5
    double m5_1 = -sin(f5) * (ax * cos(t11) * cos(t21)+ay * sin(t11) * cos(t21)+az * sin(f1) * sin(t21));
    double n5_1 = sin(f5) * (ax * cos(t11) * sin(t21)+ay * sin(t11) * sin(t21)-az * sin(f1) * cos(t21));
    double m5_2 = -sin(f5) * (ax * cos(t11) * cos(t22)+ay * sin(t11) * cos(t22)+az * sin(f1) * sin(t22));
    double n5_2 = sin(f5) * (ax * cos(t11) * sin(t22)+ay * sin(t11) * sin(t22)-az * sin(f1) * cos(t22));
    double m5_3 = -sin(f5) * (ax * cos(t12) * cos(t23)+ay * sin(t12) * cos(t23)+az * sin(f1) * sin(t23));
    double n5_3 = sin(f5) * (ax * cos(t12) * sin(t23)+ay * sin(t12) * sin(t23)-az * sin(f1) * cos(t23));
    double m5_4 = -sin(f5) * (ax * cos(t12) * cos(t24)+ay * sin(t12) * cos(t24)+az * sin(f1) * sin(t24));
    double n5_4 = sin(f5) * (ax * cos(t12) * sin(t24)+ay * sin(t12) * sin(t24)-az * sin(f1) * cos(t24));

    double t51 = atan2(sqrt(pow(ay * cos(t11)-ax * sin(t11), 2)+pow(m5_1 * cos(t31)+n5_1 * sin(t31), 2)),
                       (m5_1 * sin(t31)-n5_1 * cos(t31)) / (sin(f3) * sin(f4)));
    double t52 = atan2(-sqrt(pow(ay * cos(t11)-ax * sin(t11), 2)+pow(m5_1 * cos(t31)+n5_1 * sin(t31), 2)),
                       (m5_1 * sin(t31)-n5_1 * cos(t31)) / (sin(f3) * sin(f4)));
    double t53 = atan2(sqrt(pow(ay * cos(t11)-ax * sin(t11), 2)+pow(m5_2 * cos(t32)+n5_2 * sin(t32), 2)),
                       (m5_2 * sin(t32)-n5_2 * cos(t32)) / (sin(f3) * sin(f4)));
    double t54 = atan2(-sqrt(pow(ay * cos(t11)-ax * sin(t11), 2)+pow(m5_2 * cos(t32)+n5_2 * sin(t32), 2)),
                       (m5_2 * sin(t32)-n5_2 * cos(t32)) / (sin(f3) * sin(f4)));
    double t55 = atan2(sqrt(pow(ay * cos(t12)-ax * sin(t12), 2)+pow(m5_3 * cos(t33)+n5_3 * sin(t33), 2)),
                       (m5_3 * sin(t33)-n5_3 * cos(t33)) / (sin(f3) * sin(f4)));
    double t56 = atan2(-sqrt(pow(ay * cos(t12)-ax * sin(t12), 2)+pow(m5_3 * cos(t33)+n5_3 * sin(t33), 2)),
                       (m5_3 * sin(t33)-n5_3 * cos(t33)) / (sin(f3) * sin(f4)));
    double t57 = atan2(sqrt(pow(ay * cos(t12)-ax * sin(t12), 2)+pow(m5_4 * cos(t34)+n5_4 * sin(t34), 2)),
                       (m5_4 * sin(t34)-n5_4 * cos(t34)) / (sin(f3) * sin(f4)));
    double t58 = atan2(-sqrt(pow(ay * cos(t12)-ax * sin(t12), 2)+pow(m5_4 * cos(t34)+n5_4 * sin(t34), 2)),
                       (m5_4 * sin(t34)-n5_4 * cos(t34)) / (sin(f3) * sin(f4)));

    // 计算t4
    double t41 = sin(t51) == 0? 0 : atan2((ay * cos(t11)-ax * sin(t11)) * sin(f1) * sin(f5) / (-sin(t51) * sin(f3)),
                                           (-m5_1 * cos(t31)-n5_1 * sin(t31)) / sin(t51));
    double t42 = sin(t52) == 0? 0 : atan2((ay * cos(t11)-ax * sin(t11)) * sin(f1) * sin(f5) / (-sin(t52) * sin(f3)),
                                           (-m5_1 * cos(t31)-n5_1 * sin(t31)) / sin(t52));
    double t43 = sin(t53) == 0? 0 : atan2((ay * cos(t11)-ax * sin(t11)) * sin(f1) * sin(f5) / (-sin(t53) * sin(f3)),
                                           (-m5_2 * cos(t32)-n5_2 * sin(t32)) / sin(t53));
    double t44 = sin(t54) == 0? 0 : atan2((ay * cos(t11)-ax * sin(t11)) * sin(f1) * sin(f5) / (-sin(t54) * sin(f3)),
                                           (-m5_2 * cos(t32)-n5_2 * sin(t32)) / sin(t54));
    double t45 = sin(t55) == 0? 0 : atan2((ay * cos(t12)-ax * sin(t12)) * sin(f1) * sin(f5) / (-sin(t55) * sin(f3)),
                                           (-m5_3 * cos(t33)-n5_3 * sin(t33)) / sin(t55));
    double t46 = sin(t56) == 0? 0 : atan2((ay * cos(t12)-ax * sin(t12)) * sin(f1) * sin(f5) / (-sin(t56) * sin(f3)),
                                           (-m5_3 * cos(t33)-n5_3 * sin(t33)) / sin(t56));
    double t47 = sin(t57) == 0? 0 : atan2((ay * cos(t12)-ax * sin(t12)) * sin(f1) * sin(f5) / (-sin(t57) * sin(f3)),
                                           (-m5_4 * cos(t34)-n5_4 * sin(t34)) / sin(t57));
    double t48 = sin(t58) == 0? 0 : atan2((ay * cos(t12)-ax * sin(t12)) * sin(f1) * sin(f5) / (-sin(t58) * sin(f3)),
                                           (-m5_4 * cos(t34)-n5_4 * sin(t34)) / sin(t58));
    // // 计算t6
    // double e1 = nx * sin(t11)-ny * cos(t11);
    // double f1Tmp = ox * sin(t11)-oy * cos(t11);
    // // 修改t61到t64的计算，使用f1Tmp
    // double t61 = atan2(cos(t41) * e1 - cos(t51) * sin(t41) * f1Tmp, cos(t41) * f1Tmp + cos(t51) * sin(t41) * e1);
    // double t62 = atan2(cos(t42) * e1 - cos(t52) * sin(t42) * f1Tmp, cos(t42) * f1Tmp + cos(t52) * sin(t42) * e1);
    // double t63 = atan2(cos(t43) * e1 - cos(t53) * sin(t43) * f1Tmp, cos(t43) * f1Tmp + cos(t53) * sin(t43) * e1);
    // double t64 = atan2(cos(t44) * e1 - cos(t54) * sin(t44) * f1Tmp, cos(t44) * f1Tmp + cos(t54) * sin(t44) * e1);

    // double e2 = nx * sin(t12)-ny * cos(t12);
    // double f2 = ox * sin(t12)-oy * cos(t12);
    // double t65 = atan2(cos(t45) * e2 - cos(t55) * sin(t45) * f2, cos(t45) * f2 + cos(t55) * sin(t45) * e2);
    // double t66 = atan2(cos(t46) * e2 - cos(t56) * sin(t46) * f2, cos(t46) * f2 + cos(t56) * sin(t46) * e2);
    // double t67 = atan2(cos(t47) * e2 - cos(t57) * sin(t47) * f2, cos(t47) * f2 + cos(t57) * sin(t47) * e2);
    // double t68 = atan2(cos(t48) * e2 - cos(t58) * sin(t48) * f2, cos(t48) * f2 + cos(t58) * sin(t48) * e2);

    ikine_t[0] = {t11, t21, t31, t41};
    ikine_t[1] = {t11, t21, t31, t42};
    ikine_t[2] = {t11, t22, t32, t43};
    ikine_t[3] = {t11, t22, t32, t44};
    ikine_t[4] = {t12, t23, t33, t45};
    ikine_t[5] = {t12, t23, t33, t46};
    ikine_t[6] = {t12, t24, t34, t47};
    ikine_t[7] = {t12, t24, t34, t48};

    return ikine_t;
}

void MainWindow::onForwardSolveClicked()
{
    bool ok1, ok2, ok3, ok4;
    //ok5, ok6;
    double theta1 = theta1Edit->text().toDouble(&ok1);
    double theta2 = theta2Edit->text().toDouble(&ok2);
    double theta3 = theta3Edit->text().toDouble(&ok3);
    double theta4 = theta4Edit->text().toDouble(&ok4);

    if (!ok1 ||!ok2 ||!ok3 ||!ok4 ) {
        errorLabel->setText("关节角度输入无效，请输入数字");
        return;
    }

    QVector<QVector<double>> result = myfkine(theta1, theta2, theta3, theta4);

    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            QTableWidgetItem *item = new QTableWidgetItem(QString::number(result[i][j]));
            forwardResultTable->setItem(i, j, item);
        }
    }

    // // // 将弧度转换为角度，并更新关节旋转
    // // jointTransform->setRotationZ(qRadiansToDegrees(theta1));
    // // 更新关节变换，这里假设已有theta1 - theta6变量
    // jointTransforms[0]->setRotationZ(qRadiansToDegrees(theta1));
    // jointTransforms[1]->setRotationX(qRadiansToDegrees(theta2));
    // jointTransforms[2]->setRotationY(qRadiansToDegrees(theta3));
    // jointTransforms[3]->setRotationX(qRadiansToDegrees(theta4));
    // // 补充更新关节5和关节6的代码
    // jointTransforms[4]->setRotationZ(qRadiansToDegrees(theta5));
    // jointTransforms[5]->setRotationX(qRadiansToDegrees(theta6));

    // 更新关节变换（复用统一函数）
    QVector<double> angles = {theta1, theta2, theta3, theta4};
    updateJointTransforms(angles);

    // 更新末端执行器的位姿
    QMatrix4x4 T04_matrix(
        result[0][0], result[0][1], result[0][2], result[0][3],
        result[1][0], result[1][1], result[1][2], result[1][3],
        result[2][0], result[2][1], result[2][2], result[2][3],
        result[3][0], result[3][1], result[3][2], result[3][3]
        );
    eeTransform->setMatrix(T04_matrix);

    errorLabel->setText("");
}

void MainWindow::onInverseSolveClicked()
{
    QVector<QVector<double>> Tbe(4, QVector<double>(4));
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            QTableWidgetItem *item = poseMatrixInputTable->item(i, j);
            if (item) {
                bool ok;
                Tbe[i][j] = item->text().toDouble(&ok);
                if (!ok) {
                    errorLabel->setText("末端执行器位姿矩阵输入无效，请输入数字");
                    return;
                }
            } else {
                errorLabel->setText("末端执行器位姿矩阵输入不完整");
                return;
            }
        }
    }

    QVector<QVector<double>> result = mymodikine(Tbe);

    for (int i = 0; i < 8; ++i) {
        for (int j = 0; j < 6; ++j) {
            QTableWidgetItem *item = new QTableWidgetItem(QString::number(result[i][j]));
            inverseResultTable->setItem(i, j, item);
        }
    }
    errorLabel->setText("");
}

void MainWindow::onResetClicked()
{
    // 清空关节角度输入框
    theta1Edit->clear();
    theta2Edit->clear();
    theta3Edit->clear();
    theta4Edit->clear();

    // 清空输入输出表格
    for (int i = 0; i < poseMatrixInputTable->rowCount(); ++i) {
        for (int j = 0; j < poseMatrixInputTable->columnCount(); ++j) {
            poseMatrixInputTable->setItem(i, j, nullptr);
        }
    }

    for (int i = 0; i < forwardResultTable->rowCount(); ++i) {
        for (int j = 0; j < forwardResultTable->columnCount(); ++j) {
            forwardResultTable->setItem(i, j, nullptr);
        }
    }

    for (int i = 0; i < inverseResultTable->rowCount(); ++i) {
        for (int j = 0; j < inverseResultTable->columnCount(); ++j) {
            inverseResultTable->setItem(i, j, nullptr);
        }
    }

    // 恢复关节和连杆的初始变换
    for (int i = 0; i < jointTransforms.size(); ++i) {
        jointTransforms[i]->setMatrix(jointInitialTransforms[i]);
    }
    for (int i = 0; i < linkTransforms.size(); ++i) {
        linkTransforms[i]->setMatrix(linkInitialTransforms[i]);

        // 恢复连杆的初始长度
        Qt3DExtras::QCylinderMesh *linkMesh = qobject_cast<Qt3DExtras::QCylinderMesh*>(linkEntities[i]->components().first());
        if (linkMesh) {
            // 从QMatrix4x4中提取初始平移信息
            QVector3D start(jointInitialTransforms[i].column(3).x(), jointInitialTransforms[i].column(3).y(), jointInitialTransforms[i].column(3).z());
            QVector3D end(jointInitialTransforms[i + 1].column(3).x(), jointInitialTransforms[i + 1].column(3).y(), jointInitialTransforms[i + 1].column(3).z());
            QVector3D direction = end - start;
            float length = direction.length();
            linkMesh->setLength(length);
        }
    }

    // 重置末端执行器位置
    QMatrix4x4 initialMatrix; // 初始化为单位矩阵
    initialMatrix.setToIdentity();
    eeTransform->setMatrix(initialMatrix);  // 重置末端位置

    // 清除错误提示和状态消息
    errorLabel->clear();
    statusBar()->clearMessage();

    // 添加状态栏动画反馈（可选）
    statusBar()->showMessage("已完全复位", 2000);  // 显示2秒
    QTimer::singleShot(2000, [this](){
        statusBar()->showMessage("准备就绪");
    });

    // 强制界面刷新（针对某些Qt版本需要）
    qApp->processEvents();
}

void MainWindow::createCoordinateAxes() {
    // X轴（红色圆柱体）
    Qt3DCore::QEntity *xAxis = new Qt3DCore::QEntity(rootEntity);
    Qt3DExtras::QCylinderMesh *xMesh = new Qt3DExtras::QCylinderMesh(xAxis);
    xMesh->setRadius(0.02f);
    xMesh->setLength(2.0f); // 长度为2米
    xMesh->setRings(2);     // 简化网格

    Qt3DExtras::QPhongMaterial *xMat = new Qt3DExtras::QPhongMaterial(xAxis);
    xMat->setDiffuse(Qt::red);

    Qt3DCore::QTransform *xTransform = new Qt3DCore::QTransform(xAxis);
    xTransform->setRotationZ(90.0f); // 绕Z轴旋转90度，使圆柱体指向X轴

    xAxis->addComponent(xMesh);
    xAxis->addComponent(xMat);
    xAxis->addComponent(xTransform);

    // Y轴（绿色圆柱体）
    Qt3DCore::QEntity *yAxis = new Qt3DCore::QEntity(rootEntity);
    Qt3DExtras::QCylinderMesh *yMesh = new Qt3DExtras::QCylinderMesh(yAxis);
    yMesh->setRadius(0.02f);
    yMesh->setLength(2.0f);
    yMesh->setRings(2);

    Qt3DExtras::QPhongMaterial *yMat = new Qt3DExtras::QPhongMaterial(yAxis);
    yMat->setDiffuse(Qt::green);

    yAxis->addComponent(yMesh);
    yAxis->addComponent(yMat);
    yAxis->addComponent(new Qt3DCore::QTransform());

    // Z轴（蓝色圆柱体）
    Qt3DCore::QEntity *zAxis = new Qt3DCore::QEntity(rootEntity);
    Qt3DExtras::QCylinderMesh *zMesh = new Qt3DExtras::QCylinderMesh(zAxis);
    zMesh->setRadius(0.02f);
    zMesh->setLength(2.0f);
    zMesh->setRings(2);

    Qt3DExtras::QPhongMaterial *zMat = new Qt3DExtras::QPhongMaterial(zAxis);
    zMat->setDiffuse(Qt::blue);

    Qt3DCore::QTransform *zTransform = new Qt3DCore::QTransform(zAxis);
    zTransform->setRotationX(90.0f); // 绕X轴旋转90度，使圆柱体指向Z轴

    zAxis->addComponent(zMesh);
    zAxis->addComponent(zMat);
    zAxis->addComponent(zTransform);
}

void MainWindow::updateJointTransforms(const QVector<double>& angles) {
    if (angles.size() != 4) {
        errorLabel->setText("角度数量错误，需4个关节角度");
        return;
    }

    double theta1 = angles[0];
    double theta2 = angles[1];
    double theta3 = angles[2];
    double theta4 = angles[3];
    //double theta5 = angles[4];
    //double theta6 = angles[5];

    std::vector<QVector<QVector<double>>> jointMatrices = calculateJointMatrices(theta1, theta2, theta3, theta4);

    for (int i = 0; i < 4; ++i) {
        QVector<QVector<double>> T0i = jointMatrices[i];
        QVector3D position(T0i[0][3], T0i[1][3], T0i[2][3]);
        jointTransforms[i]->setTranslation(position);

        // 计算旋转矩阵到四元数的转换
        QMatrix3x3 rotationMatrix;
        rotationMatrix.setToIdentity(); // 先将矩阵初始化为单位矩阵
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                rotationMatrix(row, col) = T0i[row][col]; // 逐元素赋值
            }
        }
        QQuaternion rotation = QQuaternion::fromRotationMatrix(rotationMatrix);
        jointTransforms[i]->setRotation(rotation);
    }

    // 更新连杆变换
    // for (int i = 0; i < linkTransforms.size(); ++i) {
    for (int i = 0; i < 3; ++i) { // 遍历五个连杆
        int startJointIndex = i;
        int endJointIndex = i + 1;

        QVector3D start = jointTransforms[startJointIndex]->translation();
        QVector3D end = jointTransforms[endJointIndex]->translation();
        QVector3D direction = end - start;
        float length = direction.length();

        // 更新连杆长度
        Qt3DExtras::QCylinderMesh *linkMesh = qobject_cast<Qt3DExtras::QCylinderMesh*>(linkEntities[i]->components().first());
        if (linkMesh) {
            linkMesh->setLength(length);
        }

        // 计算旋转并更新连杆变换
        QVector3D up = QVector3D(0, 1, 0);
        QVector3D axis = QVector3D::crossProduct(up, direction.normalized());
        float angle = qRadiansToDegrees(qAcos(QVector3D::dotProduct(up, direction.normalized())));
        linkTransforms[i]->setTranslation(start + direction / 2);
        linkTransforms[i]->setRotation(QQuaternion::fromAxisAndAngle(axis, angle));
    }
}

void MainWindow::onInverseResultSelected(int row) {
    // 1. 检查行号有效性
    if (row < 0 || row >= inverseResultTable->rowCount()) {
        errorLabel->setText("无效的行选择");
        return;
    }

    // 2. 提取角度数据
    QVector<double> angles;
    bool isValid = true;
    for (int col = 0; col < 4; ++col) {
        QTableWidgetItem *item = inverseResultTable->item(row, col);
        if (!item) {
            isValid = false;
            break;
        }
        bool ok;
        double angle = item->text().toDouble(&ok);
        if (!ok) {
            isValid = false;
            break;
        }
        angles.append(angle);
    }

    // 3. 验证并更新关节
    if (!isValid || angles.size() != 4) {
        errorLabel->setText("逆解数据不完整或格式错误");
        return;
    }

    updateJointTransforms(angles);
    errorLabel->setText("");
}

std::vector<QVector<QVector<double>>> MainWindow::calculateJointMatrices(double theta1, double theta2, double theta3, double theta4)
{
    QVector<QVector<double>> MDH = {
        {theta1, 0, 0, 0},
        {theta2, 0, 0.325, -M_PI/2},
        {theta3, 0, 1.150, 0},
        {theta4, 1.225, 0.300, -M_PI/2}
        //{theta5, 0, 0, M_PI/2},
        //{theta6, 0, 0, -M_PI/2}
    };

    QVector<QVector<double>> T01 = {
        {cos(MDH[0][0]), -sin(MDH[0][0]), 0, MDH[0][2]},
        {cos(MDH[0][3])*sin(MDH[0][0]), cos(MDH[0][3])*cos(MDH[0][0]), -sin(MDH[0][3]), -MDH[0][1]*sin(MDH[0][3])},
        {sin(MDH[0][3])*sin(MDH[0][0]), sin(MDH[0][3])*cos(MDH[0][0]), cos(MDH[0][3]), MDH[0][1]*cos(MDH[0][3])},
        {0, 0, 0, 1}
    };

    QVector<QVector<double>> T12 = {
        {cos(MDH[1][0]), -sin(MDH[1][0]), 0, MDH[1][2]},
        {cos(MDH[1][3])*sin(MDH[1][0]), cos(MDH[1][3])*cos(MDH[1][0]), -sin(MDH[1][3]), -MDH[1][1]*sin(MDH[1][3])},
        {sin(MDH[1][3])*sin(MDH[1][0]), sin(MDH[1][3])*cos(MDH[1][0]), cos(MDH[1][3]), MDH[1][1]*cos(MDH[1][3])},
        {0, 0, 0, 1}
    };

    QVector<QVector<double>> T23 = {
        {cos(MDH[2][0]), -sin(MDH[2][0]), 0, MDH[2][2]},
        {cos(MDH[2][3])*sin(MDH[2][0]), cos(MDH[2][3])*cos(MDH[2][0]), -sin(MDH[2][3]), -MDH[2][1]*sin(MDH[2][3])},
        {sin(MDH[2][3])*sin(MDH[2][0]), sin(MDH[2][3])*cos(MDH[2][0]), cos(MDH[2][3]), MDH[2][1]*cos(MDH[2][3])},
        {0, 0, 0, 1}
    };

    QVector<QVector<double>> T34 = {
        {cos(MDH[3][0]), -sin(MDH[3][0]), 0, MDH[3][2]},
        {cos(MDH[3][3])*sin(MDH[3][0]), cos(MDH[3][3])*cos(MDH[3][0]), -sin(MDH[3][3]), -MDH[3][1]*sin(MDH[3][3])},
        {sin(MDH[3][3])*sin(MDH[3][0]), sin(MDH[3][3])*cos(MDH[3][0]), cos(MDH[3][3]), MDH[3][1]*cos(MDH[3][3])},
        {0, 0, 0, 1}
    };

    // QVector<QVector<double>> T45 = {
    //     {cos(MDH[4][0]), -sin(MDH[4][0]), 0, MDH[4][2]},
    //     {cos(MDH[4][3])*sin(MDH[4][0]), cos(MDH[4][3])*cos(MDH[4][0]), -sin(MDH[4][3]), -MDH[4][1]*sin(MDH[4][3])},
    //     {sin(MDH[4][3])*sin(MDH[4][0]), sin(MDH[4][3])*cos(MDH[4][0]), cos(MDH[4][3]), MDH[4][1]*cos(MDH[4][3])},
    //     {0, 0, 0, 1}
    // };

    // QVector<QVector<double>> T56 = {
    //     {cos(MDH[5][0]), -sin(MDH[5][0]), 0, MDH[5][2]},
    //     {cos(MDH[5][3])*sin(MDH[5][0]), cos(MDH[5][3])*cos(MDH[5][0]), -sin(MDH[5][3]), -MDH[5][1]*sin(MDH[5][3])},
    //     {sin(MDH[5][3])*sin(MDH[5][0]), sin(MDH[5][3])*cos(MDH[5][0]), cos(MDH[5][3]), MDH[5][1]*cos(MDH[5][3])},
    //     {0, 0, 0, 1}
    // };

    std::vector<QVector<QVector<double>>> jointMatrices(6);
    jointMatrices[0] = T01;
    jointMatrices[1] = multiplyMatrix(T01, T12);
    jointMatrices[2] = multiplyMatrix(jointMatrices[1], T23);
    jointMatrices[3] = multiplyMatrix(jointMatrices[2], T34);
    // jointMatrices[4] = multiplyMatrix(jointMatrices[3], T45);
    // jointMatrices[5] = multiplyMatrix(jointMatrices[4], T56);

    return jointMatrices;
}

// 放大视野按钮点击事件
void MainWindow::onZoomInClicked()
{
    float currentFOV = camera->fieldOfView();
    float newFOV = qMax(currentFOV - 5.0f, 10.0f); // 最小FOV限制为10度
    camera->setFieldOfView(newFOV);
    statusBar()->showMessage(QString("视野已放大，FOV: %1°").arg(newFOV), 2000);
}

// 缩小视野按钮点击事件
void MainWindow::onZoomOutClicked()
{
    float currentFOV = camera->fieldOfView();
    float newFOV = qMin(currentFOV + 5.0f, 120.0f); // 最大FOV限制为120度
    camera->setFieldOfView(newFOV);
    statusBar()->showMessage(QString("视野已缩小，FOV: %1°").arg(newFOV), 2000);
}
