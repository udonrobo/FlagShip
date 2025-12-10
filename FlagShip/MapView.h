#ifndef MAPVIEW_H
#define MAPVIEW_H

#include <QQuickPaintedItem>
#include <QColor>
#include <QPainter>
#include <QPen>
#include <QPointF>
#include <QList>
#include <QRectF>
#include <memory>
#include <QTimer>
#include <QUndoStack>
#include <QThread>

namespace Pathfinding {
    class Pathfinder;
}
class AddObstacleCommand;
class DeleteObstacleCommand;
class MoveObstacleCommand;
class AddWaypointCommand;
class DeleteWaypointCommand;
class MoveWaypointCommand;

// 探索ワーカースレッド用クラス
class PathfindingWorker : public QObject {
    Q_OBJECT
public:
    // 依存関係を断ち切るため、必要なデータを全て受け取る
    struct InputData {
        int w, h, res;
        float robotW, robotH;
        float safeThresh;
        qreal edgeThresh;
        int mode; // MapView::PathMode
        bool useWpField;
        bool isLoop;
        int pfMode; // MapView::PathfindingMode
        float tension;
        int iter;

        QPointF start;
        QPointF goal;
        QList<QPointF> wps;
        QList<int> wpModes; // 0 or 1
        QList<QRectF> obstacles;
    };

    explicit PathfindingWorker(const InputData& data, QObject* parent = nullptr);

public slots:
    void process();

signals:
    void progressChanged(float progress);
    void finished(const QList<QList<QPointF>>& segments, bool failed, int failIdx, QString msg);

private:
    InputData m_data;
};

class MapView : public QQuickPaintedItem
{
    Q_OBJECT
        friend class AddObstacleCommand;
    friend class DeleteObstacleCommand;
    friend class MoveObstacleCommand;
    friend class AddWaypointCommand;
    friend class DeleteWaypointCommand;
    friend class MoveWaypointCommand;

    Q_PROPERTY(QColor mapBackgroundColor READ mapBackgroundColor WRITE setMapBackgroundColor NOTIFY mapColorsChanged)
        Q_PROPERTY(QColor gridLineColor READ gridLineColor WRITE setGridLineColor NOTIFY mapColorsChanged)
        Q_PROPERTY(int resolution READ resolution WRITE setResolution NOTIFY resolutionChanged)
        Q_PROPERTY(int mapWidth READ mapWidth WRITE setMapWidth NOTIFY mapSizeChanged)
        Q_PROPERTY(int mapHeight READ mapHeight WRITE setMapHeight NOTIFY mapSizeChanged)
        Q_PROPERTY(bool showCenterCrosshair READ showCenterCrosshair WRITE setShowCenterCrosshair NOTIFY showCenterCrosshairChanged)

        Q_PROPERTY(EditMode editMode READ editMode WRITE setEditMode NOTIFY editModeChanged)
        Q_PROPERTY(bool dimensionInputsVisible READ dimensionInputsVisible NOTIFY obstacleDrawingStateChanged)
        Q_PROPERTY(qreal previewWidth READ previewWidth NOTIFY previewSizeChanged)
        Q_PROPERTY(qreal previewHeight READ previewHeight NOTIFY previewSizeChanged)
        Q_PROPERTY(qreal previewOpacity READ previewOpacity WRITE setPreviewOpacity NOTIFY previewOpacityChanged)
        Q_PROPERTY(ObstacleDrawingState obstacleDrawingState READ obstacleDrawingState NOTIFY obstacleDrawingStateChanged)
        Q_PROPERTY(qreal robotWidth READ robotWidth WRITE setRobotWidth NOTIFY robotSizeChanged)

        Q_PROPERTY(qreal robotHeight READ robotHeight WRITE setRobotHeight NOTIFY robotSizeChanged)
        Q_PROPERTY(qreal robotAngle READ robotAngle WRITE setRobotAngle NOTIFY robotAngleChanged)
        Q_PROPERTY(int selectedWaypointIndex READ selectedWaypointIndex NOTIFY selectedWaypointIndexChanged)
        Q_PROPERTY(PathMode selectedWaypointMode READ selectedWaypointMode WRITE setSelectedWaypointMode NOTIFY selectedWaypointModeChanged)
        Q_PROPERTY(float safetyThreshold READ safetyThreshold WRITE setSafetyThreshold NOTIFY safetyThresholdChanged)

        Q_PROPERTY(qreal fieldEdgeThreshold READ fieldEdgeThreshold WRITE setFieldEdgeThreshold NOTIFY fieldEdgeThresholdChanged)
        Q_PROPERTY(bool showSafetyZone READ showSafetyZone WRITE setShowSafetyZone NOTIFY showSafetyZoneChanged)
        Q_PROPERTY(QPointF widthInputPos READ widthInputPos NOTIFY dimensionPositionsChanged)
        Q_PROPERTY(QPointF heightInputPos READ heightInputPos NOTIFY dimensionPositionsChanged)

        Q_PROPERTY(PathfindingMode pathfindingMode READ pathfindingMode WRITE setPathfindingMode NOTIFY pathfindingModeChanged)
        Q_PROPERTY(float smoothingTension READ smoothingTension WRITE setSmoothingTension NOTIFY smoothingTensionChanged)
        Q_PROPERTY(int smoothingIterations READ smoothingIterations WRITE setSmoothingIterations NOTIFY smoothingIterationsChanged)
        Q_PROPERTY(int guidanceStrength READ guidanceStrength WRITE setGuidanceStrength NOTIFY guidanceStrengthChanged)
        Q_PROPERTY(bool loopPath READ loopPath WRITE setLoopPath NOTIFY loopPathChanged)

        // 進捗表示用プロパティ
        Q_PROPERTY(bool isFindingPath READ isFindingPath NOTIFY isFindingPathChanged)
        Q_PROPERTY(float searchProgress READ searchProgress NOTIFY searchProgressChanged)

public:
    explicit MapView(QQuickItem* parent = nullptr);
    ~MapView();

    enum class PathMode {
        Safe,
        Aggressive
    };
    Q_ENUM(PathMode)

        enum class PathfindingMode {
        Direct,
        WaypointStrict,
        WaypointGuided
    };
    Q_ENUM(PathfindingMode)

        enum class ObstacleDrawingState {
        Idle,
        Defining,
        Confirming
    };

    enum class EditMode {
        Waypoint,
        Obstacle,
        Erase,
        Move,
        StartPlacement,
        GoalPlacement,
        LoopStartPlacement
    };

    Q_ENUM(EditMode)
        Q_ENUM(ObstacleDrawingState)

        void paint(QPainter* painter) override;

    void setMapBackgroundColor(const QColor& color);
    QColor mapBackgroundColor() const;
    void setGridLineColor(const QColor& color);
    QColor gridLineColor() const;

    QList<QPointF> getWaypoints() const;
    QList<PathMode> getWaypointModes() const;
    QList<QRectF> getObstacles() const;
    QList<QPointF> getFoundPath() const;
    QList<QList<QPointF>> getFoundPathSegments() const;

    QPointF getStartPoint() const;
    bool hasStartPoint() const;
    QPointF getGoalPoint() const;
    bool hasGoalPoint() const;

    void setResolution(int r);
    int resolution() const;
    void setMapWidth(int w);
    int mapWidth() const;
    void setMapHeight(int h);
    int mapHeight() const;

    void setShowCenterCrosshair(bool show);
    bool showCenterCrosshair() const;

    void setEditMode(EditMode mode);
    EditMode editMode() const;

    bool dimensionInputsVisible() const;
    qreal previewWidth() const;
    qreal previewHeight() const;
    qreal previewOpacity() const;
    void setPreviewOpacity(qreal opacity);
    ObstacleDrawingState obstacleDrawingState() const;

    qreal robotWidth() const;
    void setRobotWidth(qreal w);
    qreal robotHeight() const;
    void setRobotHeight(qreal h);
    qreal robotAngle() const;
    void setRobotAngle(qreal a);

    int selectedWaypointIndex() const;
    PathMode selectedWaypointMode() const;
    void setSelectedWaypointMode(PathMode mode);

    float safetyThreshold() const;
    void setSafetyThreshold(float t);
    qreal fieldEdgeThreshold() const;
    void setFieldEdgeThreshold(qreal t);

    bool showSafetyZone() const;
    void setShowSafetyZone(bool show);

    QPointF widthInputPos() const;
    QPointF heightInputPos() const;

    PathfindingMode pathfindingMode() const;
    void setPathfindingMode(PathfindingMode mode);
    float smoothingTension() const;
    void setSmoothingTension(float t);
    int smoothingIterations() const;
    void setSmoothingIterations(int i);
    int guidanceStrength() const;
    void setGuidanceStrength(int s);
    bool loopPath() const;
    void setLoopPath(bool loop);

    // プロパティゲッター
    bool isFindingPath() const { return m_isFinding; }
    float searchProgress() const { return m_progress; }

    void loadMapData(int res, int w, int h, float robotW, float robotH,
        int smoothIter, const QString& searchMode,
        const QList<QPointF>& wps, const QList<int>& wpModes,
        const QList<QRectF>& obs, float defAngle,
        bool hasStart, const QPointF& startPos,
        bool hasGoal, const QPointF& goalPos);

public slots:
    void pan(qreal dx, qreal dy);
    void zoom(qreal factor, const QPointF& centerPos);
    void handleMapClick(const QPointF& viewPos, bool isCtrlPressed);
    void deleteSelectedWaypoint();
    void clearWaypoints();
    void resetView();
    void deleteSelectedObstacle();
    void updateMousePosition(const QPointF& viewPos);
    void mouseExited();
    void confirmObstaclePlacement();
    void cancelObstaclePlacement();
    void setObstacleDimension(bool isWidth, qreal dimension);
    void setStartPoint();
    void setGoalPoint();
    void setLoopStartPoint();
    void findPath();
    void undo();
    void redo();
    void confirmLoopModeActivation();
    void confirmNonLoopModeActivation();

    void startMoving(const QPointF& viewPos);
    void updateMoving(const QPointF& viewPos);
    void finishMoving(const QPointF& viewPos);

signals:
    void mapColorsChanged();
    void resolutionChanged();
    void mapSizeChanged();
    void showCenterCrosshairChanged();
    void editModeChanged();
    void obstacleDrawingStateChanged();
    void previewSizeChanged();
    void requestDimensionInputFocus();
    void previewOpacityChanged();
    void viewTransformChanged();
    void robotSizeChanged();
    void robotAngleChanged();
    void selectedWaypointIndexChanged();
    void selectedWaypointModeChanged();
    void safetyThresholdChanged();
    void fieldEdgeThresholdChanged();
    void showSafetyZoneChanged();
    void dimensionPositionsChanged();
    void pathfindingFailed(const QString& reason);
    void pathfindingModeChanged();
    void smoothingTensionChanged();
    void smoothingIterationsChanged();
    void guidanceStrengthChanged();
    void loopPathChanged();
    void requestLoopModeConfirmation();
    void requestNonLoopModeConfirmation();

    // 追加シグナル
    void isFindingPathChanged();
    void searchProgressChanged();

private slots:
    void onPathfindingFinished(const QList<QList<QPointF>>& segments, bool failed, int failIdx, const QString& msg);
    void onPathfindingProgress(float p);

private:
    void handleLeftClickInWaypointMode(const QPointF& worldPos, bool isCtrlPressed);
    void handleLeftClickInObstacleMode(const QPointF& worldPos, bool isCtrlPressed);
    void handleLeftClickInEraseMode(const QPointF& worldPos);
    void handleLeftClickInStartPlacementMode(const QPointF& worldPos);
    void handleLeftClickInGoalPlacementMode(const QPointF& worldPos);
    void handleLeftClickInLoopStartPlacementMode(const QPointF& worldPos);
    QPointF mapToView(const QPointF& worldPos) const;
    void drawDimLine(QPainter* painter, const QPointF& p1, const QPointF& p2, bool isHorizontal);
    void cullOutOfBoundsObjects();
    void setSelectedWaypointIndex(int index);
    void regeneratePathfinderGrid();
    void clearPathItems();

    qreal m_scale = 1.0;
    QPointF m_offset = QPointF(0, 0);
    QList<QPointF> m_wps;
    QList<PathMode> m_wpModes;
    QList<QRectF> m_obs;

    QColor m_bgColor = QColor("#21252b");
    QColor m_gridColor = QColor("#3a404b");

    int m_res = 10;
    int m_mapW = 150;
    int m_mapH = 150;
    bool m_showCrosshair = false;

    ObstacleDrawingState m_drawState = ObstacleDrawingState::Idle;
    QPointF m_snapPos;
    bool m_mouseIn = false;
    QPointF m_previewStart;
    QRectF m_previewObs;
    EditMode m_mode = EditMode::Waypoint;
    bool m_wFixed = false;
    bool m_hFixed = false;
    qreal m_previewAlpha = 1.0;

    int m_selObsIdx = -1;
    int m_selWpIdx = -1;

    qreal m_robotW = 100.0;
    qreal m_robotH = 100.0;
    qreal m_robotAng = 0.0;

    bool m_hasStart = false;
    QPointF m_start;
    bool m_hasGoal = false;
    QPointF m_goal;

    QList<QList<QPointF>> m_segs;
    float m_safeThresh = 1.5f;
    qreal m_edgeThresh = 0.0;
    bool m_showSafeZone = false;

    // メインスレッド用（C-Space表示用）
    std::unique_ptr<Pathfinding::Pathfinder> m_finder;
    QUndoStack* m_undo;

    PathfindingMode m_pfMode = PathfindingMode::WaypointStrict;
    float m_tension = 0.5f;
    int m_iter = 3;
    int m_guideStr = 0;
    bool m_isLoop = false;

    bool m_pfFail = false;
    int m_failSegIdx = -1;

    int m_moveWpIdx = -1;
    int m_moveObsIdx = -1;
    QPointF m_moveStartPos;
    QRectF m_moveStartRect;
    QPointF m_lastSnapPos;

    // スレッド管理
    bool m_isFinding = false;
    float m_progress = 0.0f;
};

#endif // MAPVIEW_H
