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
    // 末尾に追加された障害物を削除
    m_map->m_obs.removeLast();
    m_map->regeneratePathfinderGrid();
    m_map->update();
}

void AddObstacleCommand::redo()
{
    // 障害物をリストに追加
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
    // 削除対象を保存
    if (m_idx >= 0 && m_idx < m_map->m_obs.size()) {
        m_obs = m_map->m_obs.at(m_idx);
    }
    setText("delete obstacle");
}

void DeleteObstacleCommand::undo()
{
    // 削除した障害物を元の位置に戻す
    if (m_idx >= 0 && m_idx <= m_map->m_obs.size()) {
        m_map->m_obs.insert(m_idx, m_obs);
        m_map->regeneratePathfinderGrid();
        m_map->update();
    }
}

void DeleteObstacleCommand::redo()
{
    // 指定インデックスの障害物を削除
    if (m_idx >= 0 && m_idx < m_map->m_obs.size()) {
        m_map->m_obs.removeAt(m_idx);
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
    // 末尾のウェイポイントとそのモード設定を削除
    m_map->m_wps.removeLast();
    m_map->m_wpModes.removeLast();
    m_map->update();
}

void AddWaypointCommand::redo()
{
    // ウェイポイントを追加し、デフォルトモード(Safe)を設定
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
    // 削除対象の座標とモードを保存
    if (m_idx >= 0 && m_idx < m_map->m_wps.size()) {
        m_wp = m_map->m_wps.at(m_idx);
        m_mode = m_map->m_wpModes.at(m_idx);
    }
    setText("delete waypoint");
}

void DeleteWaypointCommand::undo()
{
    // 削除した情報を復元
    if (m_idx >= 0 && m_idx <= m_map->m_wps.size()) {
        m_map->m_wps.insert(m_idx, m_wp);
        m_map->m_wpModes.insert(m_idx, m_mode);
        m_map->update();
    }
}

void DeleteWaypointCommand::redo()
{
    // 指定インデックスのウェイポイント情報を削除
    if (m_idx >= 0 && m_idx < m_map->m_wps.size()) {
        m_map->m_wps.removeAt(m_idx);
        m_map->m_wpModes.removeAt(m_idx);
        m_map->update();
    }
}
