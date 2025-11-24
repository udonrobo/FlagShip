#include <QApplication>
#include <QQmlApplicationEngine>
#include <QDebug>
#include <QQmlContext>
#include <QIcon>

#include "backend.h"
#include "mapview.h"
#include "themecontroller.h"

using namespace Qt::StringLiterals;

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);

    app.setWindowIcon(QIcon(":/app_icon"));

    // QML型登録
    qmlRegisterType<MapView>("jp.co.flagship", 1, 0, "MapView");

    ThemeController theme;
    Backend backend;
    QQmlApplicationEngine engine;

    // QMLコンテキストプロパティ設定
    engine.rootContext()->setContextProperty("theme", &theme);
    engine.rootContext()->setContextProperty("backend", &backend);

    const QUrl url(u"qrc:/qt/qml/flagship/main.qml"_s);

    // QMLオブジェクト生成完了時の処理
    QObject::connect(&engine, &QQmlApplicationEngine::objectCreated,
        &app, [&](QObject* obj, const QUrl& objUrl) {
            if (!obj && url == objUrl)
                QCoreApplication::exit(-1);

            if (obj) {
                // MapViewインスタンスを取得してBackendに紐付け
                auto* map = obj->findChild<MapView*>("mapViewItem");
                if (map) {
                    qDebug() << "MapView linked to Backend.";
                    backend.setMapView(map);
                }
                else {
                    qCritical() << "MapView object 'mapViewItem' not found!";
                }
            }
        }, Qt::QueuedConnection);

    engine.load(url);
    return app.exec();
}
