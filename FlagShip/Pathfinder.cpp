#include "Pathfinder.h"
#include "mapView.h"
#include <cmath>
#include <queue>
#include <vector>
#include <QDebug>
#include <QLineF>

namespace Pathfinding {

    struct CompareNode {
        bool operator()(const Node* a, const Node* b) const {
            return *a > *b;
        }
    };

    Pathfinder::Pathfinder(MapView* map) : m_map(map)
    {
    }

    QList<QPoint> Pathfinder::findPath(const QPoint& start, const QPoint& goal, MapView::PathMode mode, float safeThresh, qreal edgeThresh, bool useWpField)
    {
        if (!m_map) return {};

        // グリッド再生成
        generateConfigurationSpace(mode, safeThresh, edgeThresh);
        if (mode == MapView::PathMode::Safe) {
            generateDistanceField();
        }

        // スタート/ゴールの有効性確認と補正
        QPoint s = start;
        if (!isGridPassable(s)) {
            s = findNearestPassable(s);
            if (s.x() == -1) return {};
        }

        QPoint g = goal;
        if (!isGridPassable(g)) {
            g = findNearestPassable(g);
            if (g.x() == -1) return {};
        }

        if (s.y() < 0 || s.y() >= m_gridH || s.x() < 0 || s.x() >= m_gridW ||
            g.y() < 0 || g.y() >= m_gridH || g.x() < 0 || g.x() >= m_gridW) {
            return {};
        }

        // A* 探索初期化
        std::priority_queue<Node*, std::vector<Node*>, CompareNode> openList;
        std::vector<std::vector<Node>> allNodes(m_gridH, std::vector<Node>(m_gridW));

        Node* startNode = &allNodes[s.y()][s.x()];
        startNode->pos = s;
        startNode->gCost = 0;
        startNode->hCost = heuristic(s, g);

        // 探索範囲の制限 (楕円コリドー)
        const int lowerBound = heuristic(s, g);
        const int limitCost = int(m_detourFact * lowerBound) + m_detourMargin * 10;

        openList.push(startNode);

        int dx[] = { 0, 0, 1, -1, 1, 1, -1, -1 };
        int dy[] = { 1, -1, 0, 0, 1, -1, 1, -1 };

        while (!openList.empty()) {
            Node* curr = openList.top();
            openList.pop();

            if (curr->pos == g) {
                QList<QPoint> path;
                Node* t = curr;
                while (t != nullptr) {
                    path.prepend(t->pos);
                    t = t->parent;
                }
                return path;
            }

            m_grid[curr->pos.y()][curr->pos.x()] = 2; // Closed

            for (int i = 0; i < 8; ++i) {
                QPoint next(curr->pos.x() + dx[i], curr->pos.y() + dy[i]);

                if (next.x() < 0 || next.x() >= m_gridW || next.y() < 0 || next.y() >= m_gridH) continue;
                if (m_grid[next.y()][next.x()] != 0) continue;

                // 斜め移動の角抜け判定
                if (i >= 4) {
                    if (m_grid[curr->pos.y()][next.x()] != 0 || m_grid[next.y()][curr->pos.x()] != 0) continue;
                }

                // 枝刈り
                if (heuristic(s, next) + heuristic(next, g) > limitCost) continue;

                Node* neighbor = &allNodes[next.y()][next.x()];
                int moveCost = (i < 4) ? 10 : 15; // 斜めは少しコスト増

                // 安全コスト (壁に近いほど高コスト)
                int penalty = 0;
                if (mode == MapView::PathMode::Safe && !m_distField.empty()) {
                    int d = m_distField[next.y()][next.x()];
                    int r = m_map->resolution();
                    if (r <= 0) r = 10;
                    const double d_mm = std::max(0, d) * double(r);
                    const double w = 5e5;
                    penalty = int(w / ((d_mm + 1.0) * (d_mm + 1.0)));
                }

                // ウェイポイント誘導コスト
                int attract = 0;
                if (useWpField && !m_wpField.empty()) {
                    int dist = m_wpField[next.y()][next.x()];
                    if (dist > 0) {
                        int r = m_map->resolution();
                        if (r <= 0) r = 10;
                        attract = dist * r;
                    }
                }

                int newG = curr->gCost + moveCost + penalty + attract;
                if (neighbor->parent == nullptr || newG < neighbor->gCost) {
                    neighbor->gCost = newG;
                    neighbor->hCost = heuristic(next, g);
                    neighbor->parent = curr;
                    neighbor->pos = next;
                    openList.push(neighbor);
                }
            }
        }

        return {};
    }

    const std::vector<std::vector<int>>& Pathfinder::getGrid() const { return m_grid; }

    QList<QPoint> Pathfinder::smoothPathStringPulling(const QList<QPoint>& path)
    {
        if (path.size() < 3) return path;

        QList<QPoint> smoothed;
        smoothed.append(path.first());

        int curr = 0;
        while (curr < path.size() - 1) {
            int next = curr + 1;
            for (int i = curr + 2; i < path.size(); ++i) {
                if (isGridCollisionFree(path[curr], path[i])) {
                    next = i;
                }
                else {
                    break;
                }
            }
            smoothed.append(path[next]);
            curr = next;
        }
        return smoothed;
    }

    QList<QPointF> Pathfinder::smoothWorldPathStringPulling(const QList<QPointF>& path) const
    {
        if (path.size() < 3) return path;

        QList<QPointF> smoothed;
        smoothed.append(path.first());

        int curr = 0;
        while (curr < path.size() - 1) {
            int next = curr + 1;
            for (int i = curr + 2; i < path.size(); ++i) {
                if (isWorldPathCollisionFree(path.at(curr), path.at(i))) {
                    next = i;
                }
                else {
                    break;
                }
            }
            smoothed.append(path.at(next));
            curr = next;
        }
        return smoothed;
    }

    QPointF Pathfinder::getCatmullRomPoint(float t, const QPointF& p0, const QPointF& p1, const QPointF& p2, const QPointF& p3, float alpha) const
    {
        auto dist = [](const QPointF& a, const QPointF& b) {
            return QLineF(a, b).length();
            };

        float t0 = 0.0f;
        float t1 = t0 + std::pow(dist(p0, p1), alpha);
        float t2 = t1 + std::pow(dist(p1, p2), alpha);
        float t3 = t2 + std::pow(dist(p2, p3), alpha);
        if (std::abs(t1 - t0) < 1e-5f) t1 += 1e-3f;
        if (std::abs(t2 - t1) < 1e-5f) t2 += 1e-3f;
        if (std::abs(t3 - t2) < 1e-5f) t3 += 1e-3f;

        float u = t1 + t * (t2 - t1);

        auto interp = [](float t, float ta, float tb, const QPointF& pa, const QPointF& pb) {
            return (tb - t) / (tb - ta) * pa + (t - ta) / (tb - ta) * pb;
            };

        QPointF A1 = interp(u, t0, t1, p0, p1);
        QPointF A2 = interp(u, t1, t2, p1, p2);
        QPointF A3 = interp(u, t2, t3, p2, p3);
        QPointF B1 = interp(u, t0, t2, A1, A2);
        QPointF B2 = interp(u, t1, t3, A2, A3);
        return interp(u, t1, t2, B1, B2);
    }

    QList<QPointF> Pathfinder::smoothPathCatmullRom(const QList<QPointF>& path, float alpha, int segRes) const
    {
        if (path.size() < 2 || segRes < 1) return path;

        QList<QPointF> pts;
        // 仮想的な始点・終点を追加して補間
        pts.append(2 * path.first() - path.at(1));
        pts.append(path);
        pts.append(2 * path.last() - path.at(path.size() - 2));

        QList<QPointF> smoothed;
        for (int i = 0; i < pts.size() - 3; ++i) {
            if (i == 0) smoothed.append(pts[i + 1]);

            for (int j = 1; j <= segRes; ++j) {
                float t = static_cast<float>(j) / segRes;
                smoothed.append(getCatmullRomPoint(t, pts[i], pts[i + 1], pts[i + 2], pts[i + 3], alpha));
            }
        }
        return smoothed;
    }

    QList<QPoint> Pathfinder::smoothPathChaikin(const QList<QPoint>& path, int iter) const
    {
        if (path.size() < 3 || iter <= 0) return path;

        QList<QPoint> curr = path;
        for (int i = 0; i < iter; ++i) {
            if (curr.size() < 3) break;
            QList<QPoint> next;
            next.append(curr.first());

            for (int j = 0; j < curr.size() - 1; ++j) {
                const QPoint& p0 = curr[j];
                const QPoint& p1 = curr[j + 1];
                next.append(QPoint(p0.x() * 0.75 + p1.x() * 0.25, p0.y() * 0.75 + p1.y() * 0.25));
                next.append(QPoint(p0.x() * 0.25 + p1.x() * 0.75, p0.y() * 0.25 + p1.y() * 0.75));
            }
            next.append(curr.last());

            // 衝突チェック
            bool collide = false;
            for (int k = 0; k < next.size() - 1; ++k) {
                if (!isGridCollisionFree(next[k], next[k + 1])) {
                    collide = true;
                    break;
                }
            }
            if (collide) return curr;
            curr = next;
        }
        return curr;
    }

    bool Pathfinder::isGridCollisionFree(const QPoint& p1, const QPoint& p2) const
    {
        int x1 = p1.x(), y1 = p1.y();
        int x2 = p2.x(), y2 = p2.y();
        int dx = std::abs(x2 - x1), dy = -std::abs(y2 - y1);
        int sx = (x1 < x2) ? 1 : -1;
        int sy = (y1 < y2) ? 1 : -1;
        int err = dx + dy;

        while (true) {
            if (x1 < 0 || x1 >= m_gridW || y1 < 0 || y1 >= m_gridH || m_grid[y1][x1] == 1) {
                return false;
            }
            if (x1 == x2 && y1 == y2) break;
            int e2 = 2 * err;
            if (e2 >= dy) {
                if (x1 == x2) break;
                err += dy; x1 += sx;
            }
            if (e2 <= dx) {
                if (y1 == y2) break;
                err += dx; y1 += sy;
            }
        }
        return true;
    }

    bool Pathfinder::isWorldPathCollisionFree(const QPointF& p1, const QPointF& p2) const
    {
        int res = m_map->resolution();
        if (res <= 0) return false;
        return isGridCollisionFree(
            QPoint(qFloor(p1.x() / res), qFloor(p1.y() / res)),
            QPoint(qFloor(p2.x() / res), qFloor(p2.y() / res))
        );
    }

    void Pathfinder::generateConfigurationSpace(MapView::PathMode mode, float safeThresh, qreal edgeThresh)
    {
        if (!m_map) return;
        m_gridW = m_map->mapWidth();
        m_gridH = m_map->mapHeight();
        m_grid.assign(m_gridH, std::vector<int>(m_gridW, 0));

        int res = m_map->resolution();
        if (res <= 0) return;

        // 障害物の膨張
        qreal inflate = qMax(m_map->robotWidth(), m_map->robotHeight()) / 2.0;
        if (mode == MapView::PathMode::Safe) {
            inflate *= safeThresh;
        }

        // 障害物配置
        for (const QRectF& r : m_map->getObstacles()) {
            QRectF infR = r.adjusted(-inflate, -inflate, inflate, inflate);
            int sx = qFloor(infR.left() / res);
            int sy = qFloor(infR.top() / res);
            int ex = qCeil(infR.right() / res);
            int ey = qCeil(infR.bottom() / res);

            for (int y = sy; y < ey; ++y) {
                if (y < 0 || y >= m_gridH) continue;
                for (int x = sx; x < ex; ++x) {
                    if (x < 0 || x >= m_gridW) continue;
                    QPointF c((x + 0.5) * res, (y + 0.5) * res);
                    if (infR.contains(c)) {
                        m_grid[y][x] = 1;
                    }
                }
            }
        }

        // マップ境界の安全マージン
        if (edgeThresh > 0) {
            int edge = qCeil(edgeThresh / res);
            if (edge > 0) {
                for (int y = 0; y < m_gridH; ++y) {
                    for (int x = 0; x < m_gridW; ++x) {
                        if (x < edge || x >= m_gridW - edge || y < edge || y >= m_gridH - edge) {
                            m_grid[y][x] = 1;
                        }
                    }
                }
            }
        }
    }

    void Pathfinder::generateDistanceField()
    {
        m_distField.assign(m_gridH, std::vector<int>(m_gridW, -1));
        std::queue<QPoint> q;
        int res = m_map->resolution();
        if (res <= 0) return;

        // 障害物セルの特定とキューへの追加
        for (int y = 0; y < m_gridH; ++y) {
            for (int x = 0; x < m_gridW; ++x) {
                bool isObs = false;
                for (const QRectF& r : m_map->getObstacles()) {
                    QRect gr(qFloor(r.left() / res), qFloor(r.top() / res), qCeil(r.width() / res), qCeil(r.height() / res));
                    if (gr.contains(QPoint(x, y))) {
                        isObs = true;
                        break;
                    }
                }
                if (isObs) {
                    q.push(QPoint(x, y));
                    m_distField[y][x] = 0;
                }
            }
        }

        int dx[] = { 0, 0, 1, -1 };
        int dy[] = { 1, -1, 0, 0 };

        while (!q.empty()) {
            QPoint p = q.front(); q.pop();
            for (int i = 0; i < 4; ++i) {
                int nx = p.x() + dx[i];
                int ny = p.y() + dy[i];
                if (nx >= 0 && nx < m_gridW && ny >= 0 && ny < m_gridH && m_distField[ny][nx] == -1) {
                    m_distField[ny][nx] = m_distField[p.y()][p.x()] + 1;
                    q.push(QPoint(nx, ny));
                }
            }
        }
    }

    void Pathfinder::generateWaypointField() {
        if (!m_map || m_gridW == 0 || m_gridH == 0) return;

        m_wpField.assign(m_gridH, std::vector<int>(m_gridW, -1));
        std::queue<QPoint> q;
        int res = m_map->resolution();
        if (res <= 0) return;

        QList<QPointF> wps = m_map->getWaypoints();
        for (const QPointF& wp : wps) {
            QPoint p(qFloor(wp.x() / res), qFloor(wp.y() / res));
            if (p.x() >= 0 && p.x() < m_gridW && p.y() >= 0 && p.y() < m_gridH && m_wpField[p.y()][p.x()] == -1) {
                m_wpField[p.y()][p.x()] = 0;
                q.push(p);
            }
        }

        int dx[] = { 0, 0, 1, -1 };
        int dy[] = { 1, -1, 0, 0 };

        while (!q.empty()) {
            QPoint p = q.front(); q.pop();
            for (int i = 0; i < 4; ++i) {
                int nx = p.x() + dx[i];
                int ny = p.y() + dy[i];
                if (nx >= 0 && nx < m_gridW && ny >= 0 && ny < m_gridH && m_wpField[ny][nx] == -1) {
                    m_wpField[ny][nx] = m_wpField[p.y()][p.x()] + 1;
                    q.push(QPoint(nx, ny));
                }
            }
        }
    }

    int Pathfinder::heuristic(const QPoint& a, const QPoint& b) const
    {
        int dx = std::abs(a.x() - b.x());
        int dy = std::abs(a.y() - b.y());
        // Octile距離
        return 10 * (dx + dy) + (14 - 2 * 10) * std::min(dx, dy);
    }

    bool Pathfinder::isGridPassable(const QPoint& p) const
    {
        if (p.x() < 0 || p.x() >= m_gridW || p.y() < 0 || p.y() >= m_gridH) return false;
        return m_grid[p.y()][p.x()] == 0;
    }

    QPoint Pathfinder::findNearestPassable(const QPoint& p) const
    {
        if (isGridPassable(p)) return p;

        std::queue<QPoint> q;
        std::vector<std::vector<bool>> visited(m_gridH, std::vector<bool>(m_gridW, false));

        if (p.x() >= 0 && p.x() < m_gridW && p.y() >= 0 && p.y() < m_gridH) {
            q.push(p);
            visited[p.y()][p.x()] = true;
        }
        else {
            qWarning() << "Start point out of bounds.";
            return QPoint(-1, -1);
        }

        int dx[] = { 0, 0, 1, -1, 1, 1, -1, -1 };
        int dy[] = { 1, -1, 0, 0, 1, -1, 1, -1 };

        while (!q.empty()) {
            QPoint curr = q.front(); q.pop();
            for (int i = 0; i < 8; ++i) {
                QPoint next(curr.x() + dx[i], curr.y() + dy[i]);
                if (next.x() >= 0 && next.x() < m_gridW && next.y() >= 0 && next.y() < m_gridH && !visited[next.y()][next.x()]) {
                    if (isGridPassable(next)) return next;
                    visited[next.y()][next.x()] = true;
                    q.push(next);
                }
            }
        }
        return QPoint(-1, -1);
    }

    QList<QPointF> Pathfinder::resampleByArcLength(const QList<QPointF>& pts, double ds) const
    {
        QList<QPointF> out;
        if (pts.size() < 2 || ds <= 0) return pts;

        QVector<double> s(pts.size(), 0.0);
        for (int i = 1; i < pts.size(); ++i) {
            s[i] = s[i - 1] + QLineF(pts[i - 1], pts[i]).length();
        }

        double total = s.last();
        if (total <= 0.0) return pts;

        out.reserve(int(total / ds) + 2);
        out.push_back(pts.first());

        double target = ds;
        int seg = 1;
        while (target < total && seg < pts.size()) {
            while (seg < pts.size() && s[seg] < target) ++seg;
            if (seg >= pts.size()) break;

            double t = (target - s[seg - 1]) / (s[seg] - s[seg - 1] + 1e-9);
            out.push_back(pts[seg - 1] + (pts[seg] - pts[seg - 1]) * t);
            target += ds;
        }
        if (out.last() != pts.last()) out.push_back(pts.last());
        return out;
    }

} // namespace Pathfinding
