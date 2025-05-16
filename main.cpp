#include "mainwindow.h"

#include <QApplication>
#include <QSurfaceFormat>
#include <QLoggingCategory>

int main(int argc, char *argv[])
{
    // 设置 OpenGL 版本
    QSurfaceFormat format;
    format.setRenderableType(QSurfaceFormat::OpenGL);
    format.setVersion(3, 2); // OpenGL 3.3 核心模式
    format.setProfile(QSurfaceFormat::CoreProfile);
    format.setOptions(QSurfaceFormat::DeprecatedFunctions); // 禁用旧函数
    format.setSamples(4); // 用抗锯齿
    QSurfaceFormat::setDefaultFormat(format);

    // 设置环境变量（可选）
    //qputenv("QSG_RHI_BACKEND", "opengl");
    qputenv("QSG_RENDER_LOOP", "basic");
    QLoggingCategory::setFilterRules("qt.rhi.*=true");

    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    return a.exec();
}
