#ifndef COMMANDS_H
#define COMMANDS_H

#include <QUndoCommand>
#include <QRectF>
#include <QPointF>
#include "mapView.h"

class MapView;

// ------------------------------------------------------------------
// 障害物追加コマンド
// ------------------------------------------------------------------
class AddObstacleCommand : public QUndoCommand
{
public:
    explicit AddObstacleCommand(MapView* map, const QRectF& obs, QUndoCommand* parent = nullptr);

    void undo() override;
    void redo() override;

private:
    MapView* m_map;
    QRectF m_obs;
};

// ------------------------------------------------------------------
// 障害物削除コマンド
// ------------------------------------------------------------------
class DeleteObstacleCommand : public QUndoCommand
{
public:
    explicit DeleteObstacleCommand(MapView* map, int idx, QUndoCommand* parent = nullptr);

    void undo() override;
    void redo() override;

private:
    MapView* m_map;
    QRectF m_obs;
    int m_idx;
};

// ------------------------------------------------------------------
// 障害物移動コマンド
// ------------------------------------------------------------------
class MoveObstacleCommand : public QUndoCommand
{
public:
    explicit MoveObstacleCommand(MapView* map, int idx, const QRectF& oldRect, const QRectF& newRect, QUndoCommand* parent = nullptr);

    void undo() override;
    void redo() override;

private:
    MapView* m_map;
    int m_idx;
    QRectF m_oldRect;
    QRectF m_newRect;
};

// ------------------------------------------------------------------
// ウェイポイント追加コマンド
// ------------------------------------------------------------------
class AddWaypointCommand : public QUndoCommand
{
public:
    explicit AddWaypointCommand(MapView* map, const QPointF& wp, QUndoCommand* parent = nullptr);

    void undo() override;
    void redo() override;

private:
    MapView* m_map;
    QPointF m_wp;
};

// ------------------------------------------------------------------
// ウェイポイント削除コマンド
// ------------------------------------------------------------------
class DeleteWaypointCommand : public QUndoCommand
{
public:
    explicit DeleteWaypointCommand(MapView* map, int idx, QUndoCommand* parent = nullptr);

    void undo() override;
    void redo() override;

private:
    MapView* m_map;
    QPointF m_wp;
    MapView::PathMode m_mode;
    int m_idx;
};

// ------------------------------------------------------------------
// ウェイポイント移動コマンド
// ------------------------------------------------------------------
class MoveWaypointCommand : public QUndoCommand
{
public:
    explicit MoveWaypointCommand(MapView* map, int idx, const QPointF& oldPos, const QPointF& newPos, QUndoCommand* parent = nullptr);

    void undo() override;
    void redo() override;

private:
    MapView* m_map;
    int m_idx;
    QPointF m_oldPos;
    QPointF m_newPos;
};

#endif // COMMANDS_H
