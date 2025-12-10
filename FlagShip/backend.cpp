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
#include <QRegularExpression>

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

    const QString rawNs = m_namespaceName;
    const QString ns = Backend::sanitizeNamespace(rawNs);
    const QString stem = Backend::sanitizeFileStem(rawNs);

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

    bool okV = false, okA = false;
    float defSpeed = speedStr.toFloat(&okV);
    float defAngle = angleStr.toFloat(&okA);
    if (!okV) defSpeed = 0.0f;
    if (!okA) defAngle = 0.0f;

    out << "#pragma once\n";
    out << "// ファイル内に致命的な変更を加えないでください。ソフトがファイルをロードできなくなる可能性があります。\n\n";
    out << "#include \"PathTypes.hpp\"\n";
    out << "#include <cstddef>\n\n";
    out << "namespace " << ns << " {\n\n";

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

    out << "// スタート・ゴール (m)\n";
    if (m_mapView->hasStartPoint()) {
        QPointF s = m_mapView->getStartPoint();
        out << "const float startPos[2] = { " << QString::number(s.x() / 1000.0, 'f', 4) << "f, "
            << QString::number(s.y() / 1000.0, 'f', 4) << "f };\n";
    }
    if (m_mapView->hasGoalPoint()) {
        QPointF g = m_mapView->getGoalPoint();
        out << "const float goalPos[2] = { " << QString::number(g.x() / 1000.0, 'f', 4) << "f, "
            << QString::number(g.y() / 1000.0, 'f', 4) << "f };\n";
    }
    out << "\n";

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

    for (int s = 0; s < segs.size(); ++s) {
        const auto& seg = segs[s];
        bool isLast = (s == segs.size() - 1);

        out << "static const size_t seg" << s << "Count = " << seg.size() << ";\n";
        out << "static const PointControlData seg" << s << "[seg" << s << "Count] = {\n";
        for (int j = 0; j < seg.size(); ++j) {

            float angle = defAngle;
            if (j + 1 < seg.size()) {
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

void Backend::loadHppFile()
{
    if (!m_mapView) return;

    QWidget* window = QApplication::activeWindow();
    const QString defPath = QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation);
    const QString path = QFileDialog::getOpenFileName(
        window,
        "Load Path Data",
        defPath,
        "C++ Header File (*.hpp);;All Files (*)"
    );
    if (path.isEmpty()) return;

    QFile file(path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        QMessageBox::critical(window, "Error", "Cannot open file for reading.");
        return;
    }

    QTextStream in(&file);
#if QT_VERSION >= QT_VERSION_CHECK(6,0,0)
    in.setEncoding(QStringConverter::Utf8);
#else
    in.setCodec("UTF-8");
#endif
    QString content = in.readAll();

    QRegularExpression reRes("const float mapRes = ([0-9.]+)f;");
    QRegularExpression reW("const int mapW = ([0-9]+);");
    QRegularExpression reH("const int mapH = ([0-9]+);");
    QRegularExpression reRobW("inline constexpr float robotW = ([0-9.]+)f;");
    QRegularExpression reRobH("inline constexpr float robotH = ([0-9.]+)f;");
    QRegularExpression reIter("const int smoothIter = ([0-9]+);");
    QRegularExpression reMode("const char\\* const searchMode = \"([^\"]+)\";");
    QRegularExpression reDefAng("const float defAngle = ([0-9.-]+)f;");

    QRegularExpression reStart("const float startPos\\[2\\] = \\{ ([0-9.-]+)f, ([0-9.-]+)f \\};");
    QRegularExpression reGoal("const float goalPos\\[2\\] = \\{ ([0-9.-]+)f, ([0-9.-]+)f \\};");

    float mapRes = 0.01f;
    int mapW = 150;
    int mapH = 150;
    float robotW = 0.1f;
    float robotH = 0.1f;
    int smoothIter = 3;
    QString searchMode = "Waypoint-Strict";
    float defAngle = 90.0f;

    bool hasStart = false;
    QPointF startPos;
    bool hasGoal = false;
    QPointF goalPos;

    auto match = reRes.match(content);
    if (match.hasMatch()) mapRes = match.captured(1).toFloat();
    match = reW.match(content);
    if (match.hasMatch()) mapW = match.captured(1).toInt();
    match = reH.match(content);
    if (match.hasMatch()) mapH = match.captured(1).toInt();
    match = reRobW.match(content);
    if (match.hasMatch()) robotW = match.captured(1).toFloat();
    match = reRobH.match(content);
    if (match.hasMatch()) robotH = match.captured(1).toFloat();
    match = reIter.match(content);
    if (match.hasMatch()) smoothIter = match.captured(1).toInt();
    match = reMode.match(content);
    if (match.hasMatch()) searchMode = match.captured(1);
    match = reDefAng.match(content);
    if (match.hasMatch()) defAngle = match.captured(1).toFloat();

    match = reStart.match(content);
    if (match.hasMatch()) {
        hasStart = true;
        startPos = QPointF(match.captured(1).toFloat() * 1000.0f, match.captured(2).toFloat() * 1000.0f);
    }

    match = reGoal.match(content);
    if (match.hasMatch()) {
        hasGoal = true;
        goalPos = QPointF(match.captured(1).toFloat() * 1000.0f, match.captured(2).toFloat() * 1000.0f);
    }

    QList<int> wpModes;
    QRegularExpression reWpModes("const int wpModes\\[.*?\\] = \\{(.*?)\\};", QRegularExpression::DotMatchesEverythingOption);
    match = reWpModes.match(content);
    if (match.hasMatch()) {
        QStringList parts = match.captured(1).split(',');
        for (const QString& p : parts) {
            wpModes.append(p.trimmed().toInt());
        }
    }

    QList<QPointF> wps;
    QRegularExpression reWps("const PointControlData wps\\[.*?\\] = \\{(.*?)\\};", QRegularExpression::DotMatchesEverythingOption);
    match = reWps.match(content);
    if (match.hasMatch()) {
        QString inner = match.captured(1);
        QRegularExpression rePt("\\{\\s*([0-9.-]+)f,\\s*([0-9.-]+)f,");
        auto it = rePt.globalMatch(inner);
        while (it.hasNext()) {
            auto m = it.next();
            float x = m.captured(1).toFloat() * 1000.0f;
            float y = m.captured(2).toFloat() * 1000.0f;
            wps.append(QPointF(x, y));
        }
    }

    QList<QRectF> obs;
    QRegularExpression reObs("const float obs\\[.*?\\] = \\{(.*?)\\};", QRegularExpression::DotMatchesEverythingOption);
    match = reObs.match(content);
    if (match.hasMatch()) {
        QString inner = match.captured(1);
        QRegularExpression reRect("\\{\\s*([0-9.-]+)f,\\s*([0-9.-]+)f,\\s*([0-9.-]+)f,\\s*([0-9.-]+)f\\s*\\}");
        auto it = reRect.globalMatch(inner);
        while (it.hasNext()) {
            auto m = it.next();
            float x = m.captured(1).toFloat() * 1000.0f;
            float y = m.captured(2).toFloat() * 1000.0f;
            float w = m.captured(3).toFloat() * 1000.0f;
            float h = m.captured(4).toFloat() * 1000.0f;
            obs.append(QRectF(x, y, w, h));
        }
    }

    m_mapView->loadMapData(
        (int)(mapRes * 1000.0f), mapW, mapH,
        robotW * 1000.0f, robotH * 1000.0f,
        smoothIter, searchMode,
        wps, wpModes, obs, defAngle,
        hasStart, startPos, hasGoal, goalPos
    );
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
