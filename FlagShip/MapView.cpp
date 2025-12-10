#include "mapview.h"
#include "Pathfinder.h"
#include "commands.h"
#include <QDebug>
#include <cmath>
#include <QPainterPath>
#include <QTimer>
#include <QUndoStack>
#include <QLineF>

namespace {
    const QList<QColor> WP_COLORS = {
        QColor(230, 25, 75),   QColor(60, 180, 75),   QColor(0, 130, 200),
        QColor(255, 225, 25),  QColor(245, 130, 48),  QColor(145, 30, 180),
        QColor(70, 240, 240),  QColor(240, 50, 230),  QColor(210, 245, 60),
        QColor(250, 190, 212)
    };
}

MapView::MapView(QQuickItem* parent) : QQuickPaintedItem(parent)
{
    m_finder = std::make_unique<Pathfinding::Pathfinder>(this);
    m_undo = new QUndoStack(this);
    QTimer::singleShot(0, this, &MapView::resetView);
}

MapView::~MapView() = default;

static QRectF getMapRect(int w, int h, int res) {
    return QRectF(0, 0, w * res, h * res);
}

void MapView::cullOutOfBoundsObjects() {
    QRectF bound = getMapRect(m_mapW, m_mapH, m_res);
    bool changed = false;

    int oldWpCount = m_wps.count();
    for (int i = oldWpCount - 1; i >= 0; --i) {
        if (!bound.contains(m_wps[i])) {
            m_wps.removeAt(i);
            m_wpModes.removeAt(i);
        }
    }
    if (m_wps.count() != oldWpCount) {
        changed = true;
        setSelectedWaypointIndex(-1);
        qDebug() << "Culled" << oldWpCount - m_wps.count() << "waypoints.";
    }

    int oldObsCount = m_obs.count();
    QList<QRectF> validObs;
    for (const QRectF& o : m_obs) {
        if (bound.intersects(o)) {
            validObs.append(o.intersected(bound));
        }
    }
    if (validObs.count() != oldObsCount) {
        changed = true;
        m_selObsIdx = -1;
        qDebug() << "Culled" << oldObsCount - validObs.count() << "obstacles.";
    }
    m_obs = validObs;

    if (m_hasStart && !bound.contains(m_start)) {
        m_hasStart = false;
        changed = true;
    }
    if (m_hasGoal && !bound.contains(m_goal)) {
        m_hasGoal = false;
        changed = true;
    }

    if (changed) {
        m_segs.clear();
        update();
    }
}

void MapView::drawDimLine(QPainter* p, const QPointF& p1, const QPointF& p2, bool horiz)
{
    const qreal off = 15.0 / m_scale;
    const qreal tick = 3.0 / m_scale;
    QPointF dir = horiz ? QPointF(0, 1) : QPointF(-1, 0);
    QPointF o1 = p1 + dir * off;
    QPointF o2 = p2 + dir * off;
    p->drawLine(o1, o2);
    p->drawLine(p1, o1 + dir * tick);
    p->drawLine(p2, o2 + dir * tick);
}

void MapView::paint(QPainter* p)
{
    p->save();
    p->scale(m_scale, m_scale);
    p->translate(-m_offset);

    QRectF bound = getMapRect(m_mapW, m_mapH, m_res);
    QRectF viewRect = QRectF(m_offset, QSizeF(width() / m_scale, height() / m_scale));

    p->fillRect(bound, m_bgColor);
    p->setPen(QPen(Qt::white, 2.0 / m_scale));
    p->drawRect(bound);

    QPen penMin(m_gridColor, 1.0 / m_scale);
    QPen penMaj(m_gridColor.lighter(150), 1.5 / m_scale);
    if (m_res > 0) {
        QRectF area = viewRect.intersected(bound);
        const int majInt = m_res * 5;

        int sx = qFloor(area.left() / m_res) * m_res;
        int ex = qCeil(area.right() / m_res) * m_res;
        for (int x = sx; x < ex; x += m_res) {
            p->setPen((x % majInt == 0) ? penMaj : penMin);
            p->drawLine(QPointF(x, area.top()), QPointF(x, area.bottom()));
        }
        int sy = qFloor(area.top() / m_res) * m_res;
        int ey = qCeil(area.bottom() / m_res) * m_res;
        for (int y = sy; y < ey; y += m_res) {
            p->setPen((y % majInt == 0) ? penMaj : penMin);
            p->drawLine(QPointF(area.left(), y), QPointF(area.right(), y));
        }
    }

    if (m_showSafeZone) {
        p->setPen(Qt::NoPen);
        p->setBrush(QColor(0, 100, 255, 40));
        const auto& grid = m_finder->getGrid();
        if (!grid.empty()) {
            for (size_t y = 0; y < grid.size(); ++y) {
                for (size_t x = 0; x < grid[0].size(); ++x) {
                    if (grid[y][x] == 1) {
                        p->drawRect(QRectF(x * m_res, y * m_res, m_res, m_res));
                    }
                }
            }
        }
    }

    if (!m_obs.isEmpty()) {
        QPainterPath path;
        for (const QRectF& r : m_obs) path.addRect(r);
        p->setPen(Qt::NoPen);
        p->setBrush(QColor(255, 80, 80, 150));
        p->drawPath(path);
    }

    if (m_selObsIdx != -1) {
        p->setPen(QPen(QColor(255, 255, 0, 220), 4.0 / m_scale));
        p->setBrush(Qt::NoBrush);
        p->drawRect(m_obs[m_selObsIdx]);
    }

    if (!m_segs.isEmpty() && !m_pfFail) {
        QPen pen(QColor(255, 165, 0), 3.0 / m_scale);
        p->setPen(pen);
        p->setBrush(Qt::NoBrush);
        for (const auto& seg : m_segs) {
            if (seg.size() < 2) continue;
            QPainterPath path;
            path.moveTo(seg.first());
            for (int i = 1; i < seg.size(); ++i) path.lineTo(seg[i]);
            p->drawPath(path);
        }
    }
    else {
        QList<QPointF> line;
        if (!m_isLoop) {
            if (m_hasStart) line.append(m_start);
            line.append(m_wps);
            if (m_hasGoal) line.append(m_goal);
        }
        else {
            line = m_wps;
        }

        if (line.count() > 1) {
            p->setPen(QPen(Qt::white, 1.0 / m_scale, Qt::DashLine));
            p->setBrush(Qt::NoBrush);
            for (int i = 0; i < line.count() - 1; ++i) {
                if (m_pfFail && i == m_failSegIdx) continue;
                p->drawLine(line[i], line[i + 1]);
            }
            if (m_isLoop) p->drawLine(line.last(), line.first());
        }
    }

    if (m_pfFail && m_failSegIdx != -1) {
        QList<QPointF> pts;
        if (m_isLoop) {
            pts = m_wps;
            pts.append(pts.first());
        }
        else {
            if (m_hasStart) pts.append(m_start);
            pts.append(m_wps);
            if (m_hasGoal) pts.append(m_goal);
        }
        if (m_failSegIdx + 1 < pts.count()) {
            p->setPen(QPen(QColor(255, 0, 0, 200), 4.0 / m_scale, Qt::DashLine));
            p->drawLine(pts[m_failSegIdx], pts[m_failSegIdx + 1]);
        }
    }

    const qreal rad = (m_res > 0 ? 0.6 * m_res : 3.0);
    QFont font = p->font();
    font.setBold(true);

    for (int i = 0; i < m_wps.count(); ++i) {
        const QPointF& pt = m_wps[i];
        const QColor& col = WP_COLORS[i % WP_COLORS.size()];
        QRectF r(pt.x() - rad, pt.y() - rad, rad * 2, rad * 2);

        if (i == m_selWpIdx) {
            p->setPen(QPen(QColor(255, 255, 0, 220), 4.0 / m_scale));
            p->setBrush(col);
            p->drawEllipse(r.adjusted(-rad * 0.2, -rad * 0.2, rad * 0.2, rad * 0.2));
        }
        else {
            p->setPen(Qt::NoPen);
            p->setBrush(col);
            p->drawEllipse(r);
        }

        font.setPointSizeF(rad * 1.1);
        p->setFont(font);
        p->setPen(Qt::white);
        p->drawText(r, Qt::AlignCenter, QString::number(i));
    }

    auto drawBot = [&](const QPointF& c, const QColor& col) {
        QRectF r(0, 0, m_robotW, m_robotH);
        r.moveCenter(c);
        p->setBrush(col);
        p->setPen(QPen(col.darker(120), 2.0 / m_scale, Qt::DashLine));
        p->drawRect(r);
        };

    if (m_mode == EditMode::StartPlacement) drawBot(m_snapPos, QColor(60, 180, 75, 100));
    else if (m_mode == EditMode::GoalPlacement) drawBot(m_snapPos, QColor(230, 25, 75, 100));
    else if (m_mode == EditMode::LoopStartPlacement) drawBot(m_snapPos, QColor(0, 255, 255, 100));

    if (m_isLoop) {
        if (!m_wps.isEmpty()) drawBot(m_wps.first(), QColor(0, 255, 255, 100));
    }
    else {
        if (m_hasStart) drawBot(m_start, QColor(60, 180, 75, 100));
        if (m_hasGoal) drawBot(m_goal, QColor(230, 25, 75, 100));
    }

    if ((m_mode == EditMode::Waypoint || m_mode == EditMode::Obstacle || m_drawState == ObstacleDrawingState::Idle || m_mode == EditMode::Move) && m_mouseIn) {
        p->setPen(Qt::NoPen);
        p->setBrush(QColor(255, 255, 255, 50));
        qreal r = m_res * 0.3;
        p->drawEllipse(m_snapPos, r, r);
    }

    if (m_drawState == ObstacleDrawingState::Defining || m_drawState == ObstacleDrawingState::Confirming) {
        QColor base = (m_drawState == ObstacleDrawingState::Defining) ? Qt::gray : QColor(255, 100, 100);
        base.setAlphaF(m_previewAlpha);
        p->setPen(QPen(base, 1.5 / m_scale, Qt::DashLine));
        p->setBrush(Qt::NoBrush);
        p->drawRect(m_previewObs);

        if (m_previewObs.width() > 0 || m_previewObs.height() > 0) {
            p->setPen(QPen(QColor(220, 220, 220), 1.0 / m_scale));
            drawDimLine(p, m_previewObs.bottomLeft(), m_previewObs.bottomRight(), true);
            drawDimLine(p, m_previewObs.topLeft(), m_previewObs.bottomLeft(), false);
        }
    }

    {
        qreal cx = bound.width() / 2.0;
        qreal cy = bound.height() / 2.0;
        p->setPen(QPen(Qt::white, 2.0 / m_scale));
        p->drawLine(QPointF(cx, 0), QPointF(cx, bound.height()));
        p->drawLine(QPointF(0, cy), QPointF(bound.width(), cy));
    }

    p->restore();

    p->setPen(Qt::white);
    p->setFont(QFont("Arial", 10));
    QString debug = QString("WPs: %1, Obs: %2, S: %3, G: %4, Loop: %5")
        .arg(m_wps.count()).arg(m_obs.count())
        .arg(m_hasStart ? "Yes" : "No").arg(m_hasGoal ? "Yes" : "No")
        .arg(m_isLoop ? "On" : "Off");
    p->drawText(QPoint(10, height() - 10), debug);
}

void MapView::setSelectedWaypointIndex(int idx) {
    if (m_selWpIdx != idx) {
        m_selWpIdx = idx;
        emit selectedWaypointIndexChanged();
        emit selectedWaypointModeChanged();
    }
}

bool MapView::dimensionInputsVisible() const {
    return m_drawState == ObstacleDrawingState::Defining || m_drawState == ObstacleDrawingState::Confirming;
}

qreal MapView::previewWidth() const { return m_previewObs.width(); }
qreal MapView::previewHeight() const { return m_previewObs.height(); }

QPointF MapView::widthInputPos() const {
    QPointF p(m_previewObs.center().x(), m_previewObs.bottom());
    return mapToView(p);
}

QPointF MapView::heightInputPos() const {
    QPointF p(m_previewObs.left(), m_previewObs.center().y());
    return mapToView(p);
}

QPointF MapView::mapToView(const QPointF& world) const {
    return (world - m_offset) * m_scale;
}

qreal MapView::previewOpacity() const { return m_previewAlpha; }
void MapView::setPreviewOpacity(qreal opacity) {
    if (m_previewAlpha != opacity) {
        m_previewAlpha = opacity;
        emit previewOpacityChanged();
        update();
    }
}

MapView::ObstacleDrawingState MapView::obstacleDrawingState() const { return m_drawState; }

void MapView::updateMousePosition(const QPointF& viewPos) {
    m_mouseIn = true;
    QPointF world = m_offset + (viewPos / m_scale);
    qreal sx = std::round(world.x() / m_res) * m_res;
    qreal sy = std::round(world.y() / m_res) * m_res;
    m_snapPos = QPointF(sx, sy);

    if (m_drawState == ObstacleDrawingState::Defining) {
        QPointF p1 = m_previewStart;
        QPointF p2 = m_snapPos;
        if (m_wFixed) {
            int sign = (p2.x() < p1.x()) ? -1 : 1;
            p2.setX(p1.x() + m_previewObs.width() * sign);
        }
        if (m_hFixed) {
            int sign = (p2.y() < p1.y()) ? -1 : 1;
            p2.setY(p1.y() + m_previewObs.height() * sign);
        }
        m_previewObs = QRectF(p1, p2).normalized();
        emit previewSizeChanged();
        emit dimensionPositionsChanged();
    }
    update();
}

void MapView::mouseExited() {
    m_mouseIn = false;
    update();
}

void MapView::handleMapClick(const QPointF& viewPos, bool ctrl) {
    QPointF world = m_offset + (viewPos / m_scale);
    switch (m_mode) {
    case EditMode::Waypoint:           handleLeftClickInWaypointMode(world, ctrl); break;
    case EditMode::Obstacle:           handleLeftClickInObstacleMode(world, ctrl); break;
    case EditMode::Erase:              handleLeftClickInEraseMode(world); break;
    case EditMode::StartPlacement:     handleLeftClickInStartPlacementMode(world); break;
    case EditMode::GoalPlacement:      handleLeftClickInGoalPlacementMode(world); break;
    case EditMode::LoopStartPlacement: handleLeftClickInLoopStartPlacementMode(world); break;
    case EditMode::Move: break;
    }
}

void MapView::handleLeftClickInWaypointMode(const QPointF& world, bool ctrl) {
    const qreal rad = (m_res > 0 ? 0.6 * m_res : 3.0);
    const qreal selRad = rad * 1.2;

    int newSel = -1;
    if (!ctrl) {
        for (int i = 0; i < m_wps.count(); ++i) {
            QPointF d = m_wps[i] - world;
            if (QPointF::dotProduct(d, d) < (selRad * selRad)) {
                newSel = i;
                break;
            }
        }
    }

    if (newSel != -1) {
        setSelectedWaypointIndex(newSel);
    }
    else {
        setSelectedWaypointIndex(-1);
        if (!getMapRect(m_mapW, m_mapH, m_res).contains(m_snapPos)) {
            qDebug() << "Cannot add waypoint outside boundaries.";
            return;
        }
        m_undo->push(new AddWaypointCommand(this, m_snapPos));
    }
    m_segs.clear();
}

void MapView::deleteSelectedWaypoint() {
    if ((m_mode == EditMode::Waypoint || m_mode == EditMode::Erase) && m_selWpIdx != -1) {
        m_undo->push(new DeleteWaypointCommand(this, m_selWpIdx));
        setSelectedWaypointIndex(-1);
    }
}

void MapView::handleLeftClickInObstacleMode(const QPointF& world, bool ctrl) {
    const qreal margin = 5.0 / m_scale;
    if (m_drawState == ObstacleDrawingState::Idle) {
        if (!ctrl) {
            for (int i = m_obs.count() - 1; i >= 0; --i) {
                QRectF hit = m_obs[i].adjusted(-margin, -margin, margin, margin);
                if (hit.contains(world)) {
                    m_selObsIdx = i;
                    update();
                    return;
                }
            }
        }
    }

    if (m_selObsIdx != -1) {
        m_selObsIdx = -1;
        update();
    }

    QPointF clickedSnap(std::round(world.x() / m_res) * m_res, std::round(world.y() / m_res) * m_res);
    if (m_drawState == ObstacleDrawingState::Idle) {
        if (!getMapRect(m_mapW, m_mapH, m_res).contains(clickedSnap)) return;
        m_wFixed = false;
        m_hFixed = false;
        m_previewStart = clickedSnap;
        m_previewObs = QRectF(m_previewStart, m_previewStart);
        m_drawState = ObstacleDrawingState::Defining;
        emit obstacleDrawingStateChanged();
        emit previewSizeChanged();
        emit dimensionPositionsChanged();
        emit requestDimensionInputFocus();
    }
    else if (m_drawState == ObstacleDrawingState::Defining) {
        m_previewObs = QRectF(m_previewStart, clickedSnap).normalized();
        m_drawState = ObstacleDrawingState::Confirming;
        emit obstacleDrawingStateChanged();
        emit requestDimensionInputFocus();
    }
    m_segs.clear();
    update();
}

void MapView::handleLeftClickInEraseMode(const QPointF& world) {
    const qreal rad = (m_res > 0 ? 0.6 * m_res : 3.0);
    const qreal selRad = rad * 1.2;

    for (int i = m_wps.count() - 1; i >= 0; --i) {
        QPointF d = m_wps[i] - world;
        if (QPointF::dotProduct(d, d) < (selRad * selRad)) {
            m_undo->push(new DeleteWaypointCommand(this, i));
            return;
        }
    }

    const qreal margin = 5.0 / m_scale;
    for (int i = m_obs.count() - 1; i >= 0; --i) {
        QRectF hit = m_obs[i].adjusted(-margin, -margin, margin, margin);
        if (hit.contains(world)) {
            m_undo->push(new DeleteObstacleCommand(this, i));
            return;
        }
    }
}

void MapView::deleteSelectedObstacle() {
    if ((m_mode == EditMode::Obstacle || m_mode == EditMode::Erase) && m_selObsIdx != -1) {
        m_undo->push(new DeleteObstacleCommand(this, m_selObsIdx));
        m_selObsIdx = -1;
    }
}

void MapView::setObstacleDimension(bool isWidth, qreal dim) {
    if (m_drawState != ObstacleDrawingState::Defining && m_drawState != ObstacleDrawingState::Confirming) return;
    if (dim < 0) return;

    QPointF p1 = m_previewObs.topLeft();
    qreal w = m_previewObs.width();
    qreal h = m_previewObs.height();
    if (isWidth) { m_wFixed = true; w = dim; }
    else { m_hFixed = true; h = dim; }

    m_previewObs = QRectF(p1, QSizeF(w, h));
    emit previewSizeChanged();
    emit dimensionPositionsChanged();
    update();
}

void MapView::confirmObstaclePlacement() {
    if (m_drawState != ObstacleDrawingState::Confirming && m_drawState != ObstacleDrawingState::Defining) return;

    m_wFixed = false;
    m_hFixed = false;
    QRectF bound = getMapRect(m_mapW, m_mapH, m_res);

    if (m_previewObs.isValid() && m_previewObs.intersects(bound)) {
        QRectF finalObs = m_previewObs.intersected(bound);
        if (finalObs.isValid()) {
            m_undo->push(new AddObstacleCommand(this, finalObs));
        }
    }

    m_drawState = ObstacleDrawingState::Idle;
    m_previewObs = QRectF();
    emit obstacleDrawingStateChanged();
    update();
}

void MapView::cancelObstaclePlacement() {
    if (m_drawState == ObstacleDrawingState::Idle) return;
    m_wFixed = false;
    m_hFixed = false;
    m_drawState = ObstacleDrawingState::Idle;
    m_previewObs = QRectF();
    if (m_selObsIdx != -1) m_selObsIdx = -1;
    setSelectedWaypointIndex(-1);
    m_pfFail = false;
    m_failSegIdx = -1;
    emit obstacleDrawingStateChanged();
    update();
}

void MapView::clearWaypoints() {
    cancelObstaclePlacement();
    m_wps.clear();
    m_wpModes.clear();
    m_obs.clear();
    m_selObsIdx = -1;
    setSelectedWaypointIndex(-1);
    m_hasStart = false;
    m_hasGoal = false;
    m_segs.clear();
    m_pfFail = false;
    m_failSegIdx = -1;
    m_undo->clear();
    regeneratePathfinderGrid();
    update();
}

void MapView::resetView() {
    cancelObstaclePlacement();
    QRectF bound = getMapRect(m_mapW, m_mapH, m_res);
    if (bound.width() <= 0 || bound.height() <= 0) return;

    qreal sx = width() / bound.width();
    qreal sy = height() / bound.height();
    m_scale = qMin(sx, sy) * 0.95;

    qreal vw = width() / m_scale;
    qreal vh = height() / m_scale;
    m_offset = QPointF((bound.width() - vw) / 2.0, (bound.height() - vh) / 2.0);
    update();
}

void MapView::pan(qreal dx, qreal dy) {
    QPointF newOff = m_offset + QPointF(dx / m_scale, dy / m_scale);
    QRectF bound = getMapRect(m_mapW, m_mapH, m_res);
    qreal vw = width() / m_scale;
    qreal vh = height() / m_scale;
    qreal mx = vw * 0.8;
    qreal my = vh * 0.8;

    if (vw <= bound.width()) newOff.setX(qBound(-mx, newOff.x(), (bound.width() - vw) + mx));
    else newOff.setX(qBound((bound.width() - vw) - mx, newOff.x(), mx));

    if (vh <= bound.height()) newOff.setY(qBound(-my, newOff.y(), (bound.height() - vh) + my));
    else newOff.setY(qBound((bound.height() - vh) - my, newOff.y(), my));

    if (m_offset != newOff) {
        m_offset = newOff;
        update();
        emit viewTransformChanged();
        emit dimensionPositionsChanged();
    }
}

void MapView::zoom(qreal factor, const QPointF& center) {
    QPointF oldOff = m_offset;
    qreal oldScale = m_scale;
    m_scale *= factor;

    QRectF bound = getMapRect(m_mapW, m_mapH, m_res);
    if (bound.width() > 0 && bound.height() > 0) {
        qreal sx = width() / bound.width();
        qreal sy = height() / bound.height();
        qreal minS = qMin(sx, sy) * 0.95;
        if (minS < 0.001) minS = 0.001;
        m_scale = qBound(minS, m_scale, 20.0);
    }
    else {
        m_scale = qBound(0.05, m_scale, 20.0);
    }

    m_offset = oldOff + center * (1.0 / oldScale - 1.0 / m_scale);
    update();
    emit viewTransformChanged();
    emit dimensionPositionsChanged();
}

void MapView::setShowCenterCrosshair(bool show) {
    if (m_showCrosshair != show) {
        m_showCrosshair = show;
        emit showCenterCrosshairChanged();
        update();
    }
}
bool MapView::showCenterCrosshair() const { return m_showCrosshair; }

void MapView::setMapWidth(int w) {
    if (m_mapW != w) {
        m_mapW = w;
        cullOutOfBoundsObjects();
        regeneratePathfinderGrid();
        emit mapSizeChanged();
        update();
    }
}
int MapView::mapWidth() const { return m_mapW; }

void MapView::setMapHeight(int h) {
    if (m_mapH != h) {
        m_mapH = h;
        cullOutOfBoundsObjects();
        regeneratePathfinderGrid();
        emit mapSizeChanged();
        update();
    }
}
int MapView::mapHeight() const { return m_mapH; }

void MapView::setResolution(int r) {
    if (m_res != r) {
        m_res = r;
        cullOutOfBoundsObjects();
        regeneratePathfinderGrid();
        emit resolutionChanged();
        update();
    }
}
int MapView::resolution() const { return m_res; }

void MapView::setEditMode(EditMode m) {
    if (m_mode != m) {
        cancelObstaclePlacement();
        m_mode = m;
        m_selObsIdx = -1;
        setSelectedWaypointIndex(-1);
        emit editModeChanged();
    }
}
MapView::EditMode MapView::editMode() const { return m_mode; }

QList<QPointF> MapView::getWaypoints() const { return m_wps; }
QList<MapView::PathMode> MapView::getWaypointModes() const { return m_wpModes; }
QList<QRectF> MapView::getObstacles() const { return m_obs; }

void MapView::setMapBackgroundColor(const QColor& c) {
    if (m_bgColor != c) {
        m_bgColor = c;
        emit mapColorsChanged();
        update();
    }
}
QColor MapView::mapBackgroundColor() const { return m_bgColor; }

void MapView::setGridLineColor(const QColor& c) {
    if (m_gridColor != c) {
        m_gridColor = c;
        emit mapColorsChanged();
        update();
    }
}
QColor MapView::gridLineColor() const { return m_gridColor; }

qreal MapView::robotWidth() const { return m_robotW; }
void MapView::setRobotWidth(qreal w) {
    if (m_robotW != w) {
        m_robotW = w;
        regeneratePathfinderGrid();
        emit robotSizeChanged();
        update();
    }
}

qreal MapView::robotHeight() const { return m_robotH; }
void MapView::setRobotHeight(qreal h) {
    if (m_robotH != h) {
        m_robotH = h;
        regeneratePathfinderGrid();
        emit robotSizeChanged();
        update();
    }
}

qreal MapView::robotAngle() const { return m_robotAng; }
void MapView::setRobotAngle(qreal a) {
    if (m_robotAng != a) {
        m_robotAng = a;
        emit robotAngleChanged();
        update();
    }
}

int MapView::selectedWaypointIndex() const { return m_selWpIdx; }

MapView::PathMode MapView::selectedWaypointMode() const {
    if (m_selWpIdx != -1 && m_selWpIdx < m_wpModes.count()) {
        return m_wpModes[m_selWpIdx];
    }
    return PathMode::Safe;
}

void MapView::setSelectedWaypointMode(PathMode m) {
    if (m_selWpIdx != -1 && m_selWpIdx < m_wpModes.count()) {
        if (m_wpModes[m_selWpIdx] != m) {
            m_wpModes[m_selWpIdx] = m;
            emit selectedWaypointModeChanged();
            update();
        }
    }
}

MapView::PathfindingMode MapView::pathfindingMode() const { return m_pfMode; }
void MapView::setPathfindingMode(PathfindingMode m) {
    if (m_pfMode != m) {
        m_pfMode = m;
        emit pathfindingModeChanged();
        m_segs.clear();
        update();
    }
}

float MapView::smoothingTension() const { return m_tension; }
void MapView::setSmoothingTension(float t) {
    if (m_tension != t) {
        m_tension = t;
        emit smoothingTensionChanged();
        m_segs.clear();
        update();
    }
}

int MapView::smoothingIterations() const { return m_iter; }
void MapView::setSmoothingIterations(int i) {
    if (m_iter != i) {
        m_iter = i;
        emit smoothingIterationsChanged();
        m_segs.clear();
        update();
    }
}

int MapView::guidanceStrength() const { return m_guideStr; }
void MapView::setGuidanceStrength(int s) {
    if (m_guideStr != s) {
        m_guideStr = s;
        emit guidanceStrengthChanged();
        m_segs.clear();
        update();
    }
}

float MapView::safetyThreshold() const { return m_safeThresh; }
void MapView::setSafetyThreshold(float t) {
    if (m_safeThresh != t) {
        m_safeThresh = t;
        m_segs.clear();
        regeneratePathfinderGrid();
        update();
        emit safetyThresholdChanged();
    }
}

qreal MapView::fieldEdgeThreshold() const { return m_edgeThresh; }
void MapView::setFieldEdgeThreshold(qreal t) {
    if (m_edgeThresh != t) {
        m_edgeThresh = t;
        m_segs.clear();
        regeneratePathfinderGrid();
        update();
        emit fieldEdgeThresholdChanged();
    }
}

bool MapView::showSafetyZone() const { return m_showSafeZone; }
void MapView::setShowSafetyZone(bool s) {
    if (m_showSafeZone != s) {
        m_showSafeZone = s;
        if (m_showSafeZone) regeneratePathfinderGrid();
        update();
        emit showSafetyZoneChanged();
    }
}

bool MapView::loopPath() const { return m_isLoop; }
void MapView::setLoopPath(bool loop) {
    if (m_isLoop == loop) return;

    if (loop) {
        if (m_wps.count() > 0 || m_hasStart || m_hasGoal) {
            emit requestLoopModeConfirmation();
            emit loopPathChanged();
        }
        else {
            m_isLoop = true;
            emit loopPathChanged();
            update();
        }
    }
    else {
        if (m_wps.count() > 0) {
            emit requestNonLoopModeConfirmation();
            emit loopPathChanged();
        }
        else {
            m_isLoop = false;
            emit loopPathChanged();
            m_segs.clear();
            update();
        }
    }
}

void MapView::clearPathItems() {
    cancelObstaclePlacement();
    m_wps.clear();
    m_wpModes.clear();
    setSelectedWaypointIndex(-1);
    m_hasStart = false;
    m_hasGoal = false;
    m_segs.clear();
    m_pfFail = false;
    m_failSegIdx = -1;
    m_undo->clear();
    update();
}

void MapView::confirmLoopModeActivation() {
    clearPathItems();
    m_isLoop = true;
    emit loopPathChanged();
    update();
}

void MapView::confirmNonLoopModeActivation() {
    clearPathItems();
    m_isLoop = false;
    emit loopPathChanged();
    update();
}

void MapView::setStartPoint() {
    setEditMode(EditMode::StartPlacement);
}

void MapView::setGoalPoint() {
    setEditMode(EditMode::GoalPlacement);
}

void MapView::setLoopStartPoint() {
    setEditMode(EditMode::LoopStartPlacement);
}

void MapView::handleLeftClickInStartPlacementMode(const QPointF&) {
    m_start = m_snapPos;
    m_hasStart = true;
    setEditMode(EditMode::Waypoint);
    m_segs.clear();
    update();
}

void MapView::handleLeftClickInGoalPlacementMode(const QPointF&) {
    m_goal = m_snapPos;
    m_hasGoal = true;
    setEditMode(EditMode::Waypoint);
    m_segs.clear();
    update();
}

void MapView::handleLeftClickInLoopStartPlacementMode(const QPointF&) {
    if (!getMapRect(m_mapW, m_mapH, m_res).contains(m_snapPos)) return;
    m_undo->push(new AddWaypointCommand(this, m_snapPos));
    setEditMode(EditMode::Waypoint);
    m_segs.clear();
    update();
}

void MapView::findPath()
{
    m_pfFail = false;
    m_failSegIdx = -1;
    m_segs.clear();
    update();

    auto gridToWorld = [&](const QList<QPoint>& gridPath) {
        QList<QPointF> world;
        if (m_res <= 0) return world;
        for (const QPoint& p : gridPath) {
            world.append(QPointF((p.x() + 0.5) * m_res, (p.y() + 0.5) * m_res));
        }
        return world;
        };

    auto handleFail = [&](int idx, bool loop) {
        m_pfFail = true;
        m_failSegIdx = idx;
        QString msg = loop
            ? QString("Loop path failed at WP %1 -> %2").arg(idx).arg((idx + 1) % m_wps.count())
            : QString("Path failed at segment %1 -> %2").arg(idx).arg(idx + 1);
        emit pathfindingFailed(msg);
        update();
        };

    const int splineRes = 30;
    QList<QPointF> pts;

    if (m_isLoop) {
        if (m_wps.count() < 2) {
            emit pathfindingFailed("Loop requires at least 2 waypoints.");
            return;
        }
        pts = m_wps;
        pts.append(m_wps.first());
    }
    else {
        if (!m_hasStart || !m_hasGoal) {
            emit pathfindingFailed("Start or Goal not set.");
            return;
        }
        pts.append(m_start);
        pts.append(m_wps);
        pts.append(m_goal);
    }

    if (pts.size() < 2) return;

    if (!m_isLoop && m_pfMode == PathfindingMode::Direct) {
        m_finder->generateWaypointField();
        QPoint sc(m_start.x() / m_res, m_start.y() / m_res);
        QPoint gc(m_goal.x() / m_res, m_goal.y() / m_res);

        auto path = m_finder->findPath(sc, gc, PathMode::Safe, m_safeThresh, m_edgeThresh, true);
        if (path.isEmpty()) {
            handleFail(0, false);
            return;
        }

        auto pulled = m_finder->smoothPathStringPulling(path);
        auto world = gridToWorld(pulled);
        if (world.size() >= 2) {
            m_segs.append(m_finder->smoothPathCatmullRom(world, m_tension, splineRes));
        }
        else {
            m_segs.append(world);
        }
    }
    else {
        QList<QList<QPointF>> ctrlSegs;
        QList<QPointF> allCtrl;

        for (int i = 0; i < pts.size() - 1; ++i) {
            QPoint s(pts[i].x() / m_res, pts[i].y() / m_res);
            QPoint g(pts[i + 1].x() / m_res, pts[i + 1].y() / m_res);

            int midx = i;
            if (m_isLoop) midx = (i < m_wps.size()) ? i : m_wps.size() - 1;
            else midx = i;

            auto mode = m_wpModes.value(midx, PathMode::Safe);
            auto path = m_finder->findPath(s, g, mode, m_safeThresh, m_edgeThresh, false);

            if (path.isEmpty()) {
                handleFail(i, m_isLoop);
                return;
            }

            auto pulled = m_finder->smoothPathStringPulling(path);
            auto world = gridToWorld(pulled);

            if (world.size() < 2) {
                world.clear();
                world.append(pts[i]);
                world.append(pts[i + 1]);
            }

            ctrlSegs.append(world);
            if (allCtrl.isEmpty()) allCtrl.append(world);
            else allCtrl.append(world.mid(1));
        }

        if (allCtrl.size() < 2) {
            m_segs = ctrlSegs;
            update();
            return;
        }

        auto smooth = m_finder->smoothPathCatmullRom(allCtrl, m_tension, splineRes);

        int startIdx = 0;
        for (int i = 0; i < ctrlSegs.size(); ++i) {
            int pairs = qMax(0, (int)ctrlSegs[i].size() - 1);
            int ptsCount = pairs * splineRes;
            int take = (startIdx == 0) ? ptsCount + 1 : ptsCount;

            if (i == ctrlSegs.size() - 1) m_segs.append(smooth.mid(startIdx));
            else m_segs.append(smooth.mid(startIdx, take));

            startIdx += ptsCount;
        }

        if (m_isLoop) {
            QList<QList<QPointF>> resampled;
            double ds = qMax(1.0, (double)m_res);
            for (const auto& s : m_segs) {
                resampled.append(m_finder->resampleByArcLength(s, ds));
            }
            m_segs = resampled;
        }
    }
    update();
}

QList<QPointF> MapView::getFoundPath() const {
    QList<QPointF> flat;
    for (const auto& s : m_segs) {
        if (flat.isEmpty()) flat.append(s);
        else if (s.count() > 1) flat.append(s.mid(1));
    }
    return flat;
}

QList<QList<QPointF>> MapView::getFoundPathSegments() const { return m_segs; }

void MapView::regeneratePathfinderGrid() {
    if (m_finder) {
        m_finder->generateConfigurationSpace(PathMode::Safe, m_safeThresh, m_edgeThresh);
    }
}

void MapView::undo() { m_undo->undo(); }
void MapView::redo() { m_undo->redo(); }

void MapView::startMoving(const QPointF& viewPos)
{
    m_moveWpIdx = -1;
    m_moveObsIdx = -1;

    QPointF world = m_offset + (viewPos / m_scale);

    const qreal rad = (m_res > 0 ? 0.6 * m_res : 3.0);
    const qreal selRad = rad * 1.5;

    for (int i = 0; i < m_wps.count(); ++i) {
        QPointF d = m_wps[i] - world;
        if (QPointF::dotProduct(d, d) < (selRad * selRad)) {
            m_moveWpIdx = i;
            m_moveStartPos = m_wps[i];
            m_lastSnapPos = m_snapPos;
            setSelectedWaypointIndex(i);
            return;
        }
    }

    const qreal margin = 5.0 / m_scale;
    for (int i = m_obs.count() - 1; i >= 0; --i) {
        QRectF hit = m_obs[i].adjusted(-margin, -margin, margin, margin);
        if (hit.contains(world)) {
            m_moveObsIdx = i;
            m_moveStartRect = m_obs[i];
            m_lastSnapPos = m_snapPos;
            m_selObsIdx = i;
            update();
            return;
        }
    }
}

void MapView::updateMoving(const QPointF& viewPos)
{
    Q_UNUSED(viewPos);

    if (m_moveWpIdx != -1) {
        QPointF delta = m_snapPos - m_lastSnapPos;
        if (delta.manhattanLength() > 0.001) {
            m_wps[m_moveWpIdx] += delta;
            m_lastSnapPos = m_snapPos;
            m_segs.clear();
            update();
        }
    }
    else if (m_moveObsIdx != -1) {
        QPointF delta = m_snapPos - m_lastSnapPos;
        if (delta.manhattanLength() > 0.001) {
            m_obs[m_moveObsIdx].translate(delta);
            m_lastSnapPos = m_snapPos;
            regeneratePathfinderGrid();
            m_segs.clear();
            update();
        }
    }
}

void MapView::finishMoving(const QPointF& viewPos)
{
    Q_UNUSED(viewPos);

    if (m_moveWpIdx != -1) {
        if (m_wps[m_moveWpIdx] != m_moveStartPos) {
            m_undo->push(new MoveWaypointCommand(this, m_moveWpIdx, m_moveStartPos, m_wps[m_moveWpIdx]));
        }
    }
    else if (m_moveObsIdx != -1) {
        if (m_obs[m_moveObsIdx] != m_moveStartRect) {
            m_undo->push(new MoveObstacleCommand(this, m_moveObsIdx, m_moveStartRect, m_obs[m_moveObsIdx]));
        }
    }

    m_moveWpIdx = -1;
    m_moveObsIdx = -1;
}
