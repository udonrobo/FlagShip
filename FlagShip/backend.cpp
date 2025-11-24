#include "backend.h"
#include "mapview.h"
#include <QFileDialog>
#include <QMessageBox>
#include <QTextStream>
#include <QStandardPaths>
#include <QApplication>
#include <QWidget>
#include <QSaveFile>
#include <QFile>
#include <QDebug>
#include <vector>
#include <string>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

Backend::Backend(QObject* parent) : QObject{ parent } {}

void Backend::setMapView(MapView* mapView) {
    m_mapView = mapView;
}

void Backend::generateHppFile(const QString& speedStr, const QString& angleStr,
    const QString& resStr, const QString& wStr,
    const QString& hStr)
{
    if (!m_mapView) {
        qCritical() << "MapView is not set in Backend!";
        QMessageBox::critical(nullptr, "Error", "Internal error: MapView not found.");
        return;
    }

    // 名前空間とファイル名幹
    const QString rawNs = m_namespaceName;
    const QString ns = Backend::sanitizeNamespace(rawNs);
    const QString stem = Backend::sanitizeFileStem(rawNs);

    // 保存先選択
    const QString defPath = QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation);
    QWidget* window = QApplication::activeWindow();
    const QString path = QFileDialog::getSaveFileName(
        window,
        "Save Path Data",
        defPath + "/" + stem + ".hpp",
        "C++ Header File (*.hpp);;All Files (*)"
    );
    if (path.isEmpty()) return;

    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QMessageBox::critical(window, "Error",
            QString("Cannot open file for writing: ") + file.errorString());
        return;
    }
    QTextStream out(&file);
#if QT_VERSION >= QT_VERSION_CHECK(6,0,0)
    out.setEncoding(QStringConverter::Utf8);
#else
    out.setCodec("UTF-8");
#endif

    // データ取得
    const auto wps = m_mapView->getWaypoints();
    const auto modes = m_mapView->getWaypointModes();
    const auto obs = m_mapView->getObstacles();
    const auto segs = m_mapView->getFoundPathSegments();
    const double rWidth = m_mapView->robotWidth();
    const double rHeight = m_mapView->robotHeight();
    const auto pfMode = m_mapView->pathfindingMode();
    const int iter = m_mapView->smoothingIterations();
    const int res = m_mapView->resolution();
    const int mapW = m_mapView->mapWidth();
    const int mapH = m_mapView->mapHeight();

    // デフォルト値設定
    bool okV = false, okA = false;
    float defSpeed = speedStr.toFloat(&okV);
    float defAngle = angleStr.toFloat(&okA);
    if (!okV) defSpeed = 0.0f;
    if (!okA) defAngle = 0.0f;

    // ヘッダ書き出し
    out << "#pragma once\n";
    out << "#include \"PathTypes.hpp\"\n";
    out << "#include <cstddef>\n\n";
    out << "namespace " << ns << " {\n\n";

    // 生成パラメータ
    QString modeStr;
    switch (pfMode) {
    case MapView::PathfindingMode::Direct:         modeStr = "Direct"; break;
    case MapView::PathfindingMode::WaypointStrict: modeStr = "Waypoint-Strict"; break;
    case MapView::PathfindingMode::WaypointGuided: modeStr = "Waypoint-Guided"; break;
    }
    out << "// 探索モード\n";
    out << "const char* const searchMode = \"" << modeStr << "\";\n";
    out << "const int smoothIter = " << iter << ";\n\n";

    out << "// ロボット寸法 (m)\n";
    out << "inline constexpr float robotW = " << QString::number(rWidth / 1000.0, 'f', 4) << "f;\n";
    out << "inline constexpr float robotH = " << QString::number(rHeight / 1000.0, 'f', 4) << "f;\n\n";

    // ウェイポイント情報
    out << "// 経路モード (0=Safe, 1=Aggressive)\n";
    out << "const size_t modeCount = " << modes.size() << ";\n";
    out << "const int wpModes[modeCount] = {";
    for (int i = 0; i < modes.size(); ++i) {
        out << (modes[i] == MapView::PathMode::Safe ? "0" : "1");
        if (i + 1 < modes.size()) out << ", ";
    }
    out << "};\n\n";

    out << "// ウェイポイント座標 (m)\n";
    out << "const size_t wpCount = " << wps.size() << ";\n";
    out << "const PointControlData wps[wpCount] = {\n";
    for (int i = 0; i < wps.size(); ++i) {
        out << "    { " << QString::number(wps[i].x() / 1000.0, 'f', 4) << "f, "
            << QString::number(wps[i].y() / 1000.0, 'f', 4) << "f, "
            << QString::number(defAngle, 'f', 2) << "f, "
            << "0.06f, 999.0f }";
        out << (i + 1 < wps.size() ? ",\n" : "\n");
    }
    out << "};\n\n";

    // 経路データ (m)
    out << "// 生成経路 (セグメント分割)\n";
    for (int s = 0; s < segs.size(); ++s) {
        const auto& seg = segs[s];
        bool isLast = (s == segs.size() - 1);

        out << "static const size_t seg" << s << "Count = " << seg.size() << ";\n";
        out << "static const PointControlData seg" << s << "[seg" << s << "Count] = {\n";
        for (int j = 0; j < seg.size(); ++j) {

            float angle = defAngle;
            if (j + 1 < seg.size()) {
                // 次点への角度計算 (北=0, 東=90)
                const QPointF& p1 = seg[j];
                const QPointF& p2 = seg[j + 1];
                double dx = p2.x() - p1.x();
                double dy = p2.y() - p1.y();
                double rad = std::atan2(dx, dy);
                angle = static_cast<float>(rad * 180.0 / M_PI);
            }

            out << "    { " << QString::number(seg[j].x() / 1000.0, 'f', 4) << "f, "
                << QString::number(seg[j].y() / 1000.0, 'f', 4) << "f, "
                << QString::number(angle, 'f', 2) << "f, "
                << "0.06f";

            // 終端セグメントの重み付け処理
            if (isLast) {
                float weight = 0.0f;
                if (seg.size() > 1) {
                    float progress = static_cast<float>(j) / static_cast<float>(seg.size() - 1);
                    weight = 999.0f * (1.0f - progress);
                }
                out << ", " << QString::number(weight, 'f', 2) << "f }";
            }
            else {
                out << " }";
            }

            out << (j + 1 < seg.size() ? ",\n" : "\n");
        }
        out << "};\n\n";
    }

    // セグメント管理配列
    out << "// セグメント検索用\n";
    out << "static const size_t segTotal = " << segs.size() << ";\n\n";

    out << "static const size_t segCounts[segTotal] = {\n";
    for (int s = 0; s < segs.size(); ++s) {
        out << "    seg" << s << "Count";
        out << (s + 1 < segs.size() ? ",\n" : "\n");
    }
    out << "};\n\n";

    out << "static const PointControlData* const segments[segTotal] = {\n";
    for (int s = 0; s < segs.size(); ++s) {
        out << "    seg" << s;
        out << (s + 1 < segs.size() ? ",\n" : "\n");
    }
    out << "};\n\n";

    out << "// 公開インスタンス\n";
    out << "inline const PathDefinition path = {\n";
    out << "    segTotal,\n";
    out << "    segCounts,\n";
    out << "    segments\n";
    out << "};\n\n";

    // 障害物・マップ情報
    out << "// 障害物 [x, y, w, h] (m)\n";
    out << "const size_t obsCount = " << obs.size() << ";\n";
    out << "const float obs[obsCount][4] = {\n";
    for (int i = 0; i < obs.size(); ++i) {
        const QRectF& r = obs[i];
        out << "    { "
            << QString::number(r.left() / 1000.0, 'f', 4) << "f, "
            << QString::number(r.top() / 1000.0, 'f', 4) << "f, "
            << QString::number(r.width() / 1000.0, 'f', 4) << "f, "
            << QString::number(r.height() / 1000.0, 'f', 4) << "f }";
        out << (i + 1 < obs.size() ? ",\n" : "\n");
    }
    out << "};\n\n";

    out << "// マップ情報\n";
    out << "const float mapRes = " << QString::number(res / 1000.0, 'f', 4) << "f;\n";
    out << "const int mapW = " << mapW << ";\n";
    out << "const int mapH = " << mapH << ";\n\n";

    out << "// デフォルト設定\n";
    out << "const float defSpeed = " << QString::number(defSpeed / 1000.0, 'f', 3) << "f;\n";
    out << "const float defAngle = " << QString::number(defAngle, 'f', 1) << "f;\n\n";

    out << "} // namespace " << ns << "\n";
}

QString Backend::sanitizeNamespace(const QString& str) {
    QString s = str.trimmed();
    if (s.isEmpty()) return QStringLiteral("PathData");
    QString out; out.reserve(s.size());
    for (const QChar& ch : s) {
        if (ch.isLetterOrNumber() || ch == QChar('_')) out.append(ch);
        else out.append('_');
    }
    if (!out.isEmpty() && out.at(0).isDigit()) out.prepend('_');
    return out;
}

QString Backend::sanitizeFileStem(const QString& str) {
    QString s = str.trimmed();
    if (s.isEmpty()) return QStringLiteral("PathData");
    QString out; out.reserve(s.size());
    for (const QChar& ch : s) {
        if (ch.isLetterOrNumber() || ch == QChar('_') || ch == QChar('-')) out.append(ch);
        else out.append('_');
    }
    if (!out.isEmpty() && out.at(0) == QChar('.')) out.prepend('_');
    return out;
}
