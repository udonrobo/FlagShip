#ifndef PATHFINDER_H
#define PATHFINDER_H

#include <QPoint>
#include <QPointF>
#include <QList>
#include <vector>
#include <functional>
#include <QRectF>

namespace Pathfinding {

    // 経路探索に必要なデータをまとめた構造体
    struct PathfinderConfig {
        int mapW;
        int mapH;
        int resolution;
        float robotW;
        float robotH;
        QList<QRectF> obstacles;
        QList<QPointF> waypoints;

        // パラメータ
        int mode; // 0:Safe, 1:Aggressive
        float safeThresh;
        qreal edgeThresh;

        // 探索制御
        bool useWpField;
        double detourFact = 1.6;
        int detourMargin = 8;
    };

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
        explicit Pathfinder();

        // セットアップ
        void setConfig(const PathfinderConfig& config);

        // 経路探索
        // progressCallback: 0.0 ~ 1.0 の進捗を通知する関数
        QList<QPoint> findPath(const QPoint& start, const QPoint& goal, std::function<void(float)> progressCallback = nullptr);

        // C-Space (障害物設定空間) の生成
        void generateConfigurationSpace();

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

    private:
        // 安全距離場の生成
        void generateDistanceField();

        int heuristic(const QPoint& a, const QPoint& b) const;
        bool isGridCollisionFree(const QPoint& p1, const QPoint& p2) const;
        bool isWorldPathCollisionFree(const QPointF& p1, const QPointF& p2) const;
        QPointF getCatmullRomPoint(float t, const QPointF& p0, const QPointF& p1, const QPointF& p2, const QPointF& p3, float alpha) const;

        PathfinderConfig m_cfg;
        int m_gridW = 0;
        int m_gridH = 0;

        // グリッドデータ (0:通行可, 1:障害物, 2:Closed)
        std::vector<std::vector<int>> m_grid;
        std::vector<std::vector<int>> m_distField;
        std::vector<std::vector<int>> m_wpField;
    };

}

#endif // PATHFINDER_H
