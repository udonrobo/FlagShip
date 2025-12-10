#ifndef BACKEND_H
#define BACKEND_H

#include <QObject>
#include <QString>

class MapView;

class Backend : public QObject
{
    Q_OBJECT
        // QMLからアクセスする名前空間名
        Q_PROPERTY(QString namespaceName READ namespaceName WRITE setNamespaceName NOTIFY namespaceNameChanged)

public:
    explicit Backend(QObject* parent = nullptr);

    // MapViewのポインタをセット
    void setMapView(MapView* mapView);

    // 名前空間の取得
    QString namespaceName() const { return m_namespaceName; }

    // 名前空間の設定（変更時のみシグナル発火）
    void setNamespaceName(const QString& ns) {
        if (m_namespaceName == ns) return;
        m_namespaceName = ns.trimmed();
        emit namespaceNameChanged();
    }

    // 文字列のサニタイズ（無効文字の置換）
    static QString sanitizeNamespace(const QString& in);
    static QString sanitizeFileStem(const QString& in);

public slots:
    // ヘッダファイル生成処理
    void generateHppFile(const QString& speedStr, const QString& angleStr,
        const QString& resStr, const QString& wStr,
        const QString& hStr);

    // ヘッダファイル読み込み処理
    void loadHppFile();

signals:
    void namespaceNameChanged();

private:
    MapView* m_mapView = nullptr;
    QString  m_namespaceName = QStringLiteral("PathData");
};

#endif // BACKEND_H
