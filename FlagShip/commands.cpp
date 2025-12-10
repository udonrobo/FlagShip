#include "commands.h"
#include "mapView.h"

// ------------------------------------------------------------------
// 障害物追加コマンド
// ------------------------------------------------------------------
AddObstacleCommand::AddObstacleCommand(MapView* map, const QRectF& obs, QUndoCommand* parent)
    : QUndoCommand(parent), m_map(map), m_obs(obs)
{
    setText("add obstacle");
}

void AddObstacleCommand::undo()
{
    m_map->m_obs.removeLast();
    m_map->regeneratePathfinderGrid();
    m_map->update();
}

void AddObstacleCommand::redo()
{
    m_map->m_obs.append(m_obs);
    m_map->regeneratePathfinderGrid();
    m_map->update();
}

// ------------------------------------------------------------------
// 障害物削除コマンド
// ------------------------------------------------------------------
DeleteObstacleCommand::DeleteObstacleCommand(MapView* map, int idx, QUndoCommand* parent)
    : QUndoCommand(parent), m_map(map), m_idx(idx)
{
    if (m_idx >= 0 && m_idx < m_map->m_obs.size()) {
        m_obs = m_map->m_obs.at(m_idx);
    }
    setText("delete obstacle");
}

void DeleteObstacleCommand::undo()
{
    if (m_idx >= 0 && m_idx <= m_map->m_obs.size()) {
        m_map->m_obs.insert(m_idx, m_obs);
        m_map->regeneratePathfinderGrid();
        m_map->update();
    }
}

void DeleteObstacleCommand::redo()
{
    if (m_idx >= 0 && m_idx < m_map->m_obs.size()) {
        m_map->m_obs.removeAt(m_idx);
        m_map->regeneratePathfinderGrid();
        m_map->update();
    }
}

// ------------------------------------------------------------------
// 障害物移動コマンド
// ------------------------------------------------------------------
MoveObstacleCommand::MoveObstacleCommand(MapView* map, int idx, const QRectF& oldRect, const QRectF& newRect, QUndoCommand* parent)
    : QUndoCommand(parent), m_map(map), m_idx(idx), m_oldRect(oldRect), m_newRect(newRect)
{
    setText("move obstacle");
}

void MoveObstacleCommand::undo()
{
    if (m_idx >= 0 && m_idx < m_map->m_obs.size()) {
        m_map->m_obs[m_idx] = m_oldRect;
        m_map->regeneratePathfinderGrid();
        m_map->update();
    }
}

void MoveObstacleCommand::redo()
{
    if (m_idx >= 0 && m_idx < m_map->m_obs.size()) {
        m_map->m_obs[m_idx] = m_newRect;
        m_map->regeneratePathfinderGrid();
        m_map->update();
    }
}

// ------------------------------------------------------------------
// ウェイポイント追加コマンド
// ------------------------------------------------------------------
AddWaypointCommand::AddWaypointCommand(MapView* map, const QPointF& wp, QUndoCommand* parent)
    : QUndoCommand(parent), m_map(map), m_wp(wp)
{
    setText("add waypoint");
}

void AddWaypointCommand::undo()
{
    m_map->m_wps.removeLast();
    m_map->m_wpModes.removeLast();
    m_map->update();
}

void AddWaypointCommand::redo()
{
    m_map->m_wps.append(m_wp);
    m_map->m_wpModes.append(MapView::PathMode::Safe);
    m_map->update();
}

// ------------------------------------------------------------------
// ウェイポイント削除コマンド
// ------------------------------------------------------------------
DeleteWaypointCommand::DeleteWaypointCommand(MapView* map, int idx, QUndoCommand* parent)
    : QUndoCommand(parent), m_map(map), m_idx(idx)
{
    if (m_idx >= 0 && m_idx < m_map->m_wps.size()) {
        m_wp = m_map->m_wps.at(m_idx);
        m_mode = m_map->m_wpModes.at(m_idx);
    }
    setText("delete waypoint");
}

void DeleteWaypointCommand::undo()
{
    if (m_idx >= 0 && m_idx <= m_map->m_wps.size()) {
        m_map->m_wps.insert(m_idx, m_wp);
        m_map->m_wpModes.insert(m_idx, m_mode);
        m_map->update();
    }
}

void DeleteWaypointCommand::redo()
{
    if (m_idx >= 0 && m_idx < m_map->m_wps.size()) {
        m_map->m_wps.removeAt(m_idx);
        m_map->m_wpModes.removeAt(m_idx);
        m_map->update();
    }
}

// ------------------------------------------------------------------
// ウェイポイント移動コマンド
// ------------------------------------------------------------------
MoveWaypointCommand::MoveWaypointCommand(MapView* map, int idx, const QPointF& oldPos, const QPointF& newPos, QUndoCommand* parent)
    : QUndoCommand(parent), m_map(map), m_idx(idx), m_oldPos(oldPos), m_newPos(newPos)
{
    setText("move waypoint");
}

void MoveWaypointCommand::undo()
{
    if (m_idx >= 0 && m_idx < m_map->m_wps.size()) {
        m_map->m_wps[m_idx] = m_oldPos;
        m_map->update();
    }
}

void MoveWaypointCommand::redo()
{
    if (m_idx >= 0 && m_idx < m_map->m_wps.size()) {
        m_map->m_wps[m_idx] = m_newPos;
        m_map->update();
    }
}
