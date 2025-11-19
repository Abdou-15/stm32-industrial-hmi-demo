#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include "serialmanger.h"

int main(int argc, char *argv[])
{
    qputenv("QT_IM_MODULE", QByteArray("qtvirtualkeyboard"));

    QGuiApplication app(argc, argv);

    // Register SerialManger as a QML type
    qmlRegisterType<SerialManger>("STM32Backend", 1, 0, "SerialManger");

    QQmlApplicationEngine engine;

    QObject::connect(
        &engine,
        &QQmlApplicationEngine::objectCreationFailed,
        &app,
        []() { QCoreApplication::exit(-1); },
        Qt::QueuedConnection);

    engine.loadFromModule("STM32_HMI_Industry", "Main");

    return app.exec();
}
