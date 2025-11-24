#ifndef PATHFINDER_H
#define PATHFINDER_H

#include <QPoint>
#include <QPointF>
#include <QList>
#include <vector>
#include "mapView.h" 

class MapView;
namespace Pathfinding {

    struct Node {
        QPoint pos;
        int gCost = 0;
        int hCost = 0;
        Node* parent = nullptr;

        int fCost() const {
            return gCost + hCost;
        }

        bool operator>(const Node& other) const {
            if (fCost() == other.fCost()) {
                return hCost > other.hCost;
            }
            return fCost() > other.fCost();
        }
    };

    class Pathfinder
    {
    public:
        explicit Pathfinder(MapView* map);

        // 経路探索
        QList<QPoint> findPath(const QPoint& start, const QPoint& goal, MapView::PathMode mode, float safeThresh, qreal edgeThresh, bool useWpField);

        // C-Space (障害物設定空間) の生成
        void generateConfigurationSpace(MapView::PathMode mode, float safeThresh, qreal edgeThresh);

        // ウェイポイント誘導場の生成
        void generateWaypointField();

        const std::vector<std::vector<int>>& getGrid() const;
        bool isGridPassable(const QPoint& p) const;
        QPoint findNearestPassable(const QPoint& p) const;

        // パス平滑化処理
        QList<QPoint> smoothPathStringPulling(const QList<QPoint>& path);
        QList<QPointF> smoothPathCatmullRom(const QList<QPointF>& path, float alpha, int segRes) const;
        QList<QPoint> smoothPathChaikin(const QList<QPoint>& path, int iter) const;
        QList<QPointF> smoothWorldPathStringPulling(const QList<QPointF>& worldPath) const;
        QList<QPointF> resampleByArcLength(const QList<QPointF>& pts, double ds) const;

        void setDetourFactor(double f) { m_detourFact = f; }
        void setDetourMarginCells(int m) { m_detourMargin = m; }

    private:
        // 安全距離場の生成
        void generateDistanceField();

        int heuristic(const QPoint& a, const QPoint& b) const;
        bool isGridCollisionFree(const QPoint& p1, const QPoint& p2) const;
        bool isWorldPathCollisionFree(const QPointF& p1, const QPointF& p2) const;
        QPointF getCatmullRomPoint(float t, const QPointF& p0, const QPointF& p1, const QPointF& p2, const QPointF& p3, float alpha) const;

        MapView* m_map;
        int m_gridW = 0;
        int m_gridH = 0;

        // グリッドデータ (0:通行可, 1:障害物, 2:Closed)
        std::vector<std::vector<int>> m_grid;
        std::vector<std::vector<int>> m_distField;
        std::vector<std::vector<int>> m_wpField;

        double m_detourFact = 1.6;   // 許容迂回係数
        int    m_detourMargin = 8;   // マージン(セル数)
    };

}

#endif // PATHFINDER_H
