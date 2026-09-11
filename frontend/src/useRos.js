/**
 * useRos.js — Live ROS 2 bridge hook
 * ------------------------------------
 * Connects to rosbridge_websocket (default ws://localhost:9090).
 * Subscribes to fleet topics and returns live state shaped
 * to match the existing App.jsx component props exactly.
 *
 * Topics consumed:
 *   /fleet/robot_states  (fleet_interfaces/msg/RobotState)
 *   /fleet/task_pool     (fleet_interfaces/msg/TaskPool)
 *   /fleet/task_events   (fleet_interfaces/msg/Task)
 *   /fleet/bundles       (fleet_interfaces/msg/Bundle)
 *   /fleet/world_views   (fleet_interfaces/msg/WorldView)
 */

import { useEffect, useRef, useState, useCallback } from 'react';

const WS_URL = import.meta.env.VITE_ROSBRIDGE_URL || 'ws://localhost:9090';

// ── Coordinate mapping ──────────────────────────────────────────────────────
// Maps ROS map-frame metres ↔ SVG pixel space for logistics_warehouse.
// Parameters match src/virtual_lab/maps/logistics_warehouse.yaml & /map topic:
//   origin: [-5.807, -7.229, 0], resolution: 0.050, size: 485 × 484
export const MAP_CONFIG = {
  originX: -5.807,      // ROS map origin x (metres)
  originY: -7.229,      // ROS map origin y (metres)
  resolution: 0.05,     // metres per pixel
  width: 485,           // image width px
  height: 484,          // image height px
};

export function rosToSvg(rx, ry) {
  // rx = originX + x * resolution  =>  x = (rx - originX) / resolution
  const x = (rx - MAP_CONFIG.originX) / MAP_CONFIG.resolution;
  // In ROS OccupancyGrid, y = originY corresponds to the bottom row (height).
  // Top row (y = 0 in SVG) corresponds to y = originY + height * resolution.
  const y = MAP_CONFIG.height - (ry - MAP_CONFIG.originY) / MAP_CONFIG.resolution;
  return {
    x: Math.round(x * 10) / 10,
    y: Math.round(y * 10) / 10,
  };
}

export function svgToRos(px, py) {
  const rx = MAP_CONFIG.originX + px * MAP_CONFIG.resolution;
  const ry = MAP_CONFIG.originY + (MAP_CONFIG.height - py) * MAP_CONFIG.resolution;
  return {
    x: Math.round(rx * 1000) / 1000,
    y: Math.round(ry * 1000) / 1000,
  };
}

// ── Robot colour palette (cycles for each unique robot_id) ──────────────────
const ROBOT_PALETTE = [
  { color: '#3978b7', colorDim: '#e4edf7' },
  { color: '#7b61a8', colorDim: '#eee9f6' },
  { color: '#c98a2e', colorDim: '#f7eddb' },
  { color: '#2b9aa0', colorDim: '#e1f1f2' },
  { color: '#c0453b', colorDim: '#f6e4e1' },
  { color: '#2f6f5e', colorDim: '#e4efe9' },
];

function robotColor(index) {
  return ROBOT_PALETTE[index % ROBOT_PALETTE.length];
}

// ── Task state → UI display ─────────────────────────────────────────────────
const STATE_LABELS = {
  0: 'Available',
  1: 'Assigned',
  2: 'In progress',
  3: 'Completed',
  4: 'Failed',
  5: 'Cancelled',
  6: 'Pickup done',
  7: 'Delivering',
};

const TASK_STATE_RANK = {
  0: 0, // Available
  1: 1, // Assigned / Allocated
  2: 2, // In progress
  6: 3, // Pickup done
  7: 4, // Delivering
  3: 5, // Completed
  4: 5, // Failed
  5: 5, // Cancelled
};

function getTaskRank(state) {
  return TASK_STATE_RANK[state] ?? Number(state);
}

function taskProgress(state) {
  return { 0: 0, 1: 10, 2: 35, 3: 100, 4: 0, 5: 0, 6: 60, 7: 80 }[state] ?? 0;
}

// ── roslib-style JSON-RPC over WebSocket (no npm dep needed) ────────────────
class RosBridge {
  constructor(url) {
    this.url = url;
    this.ws = null;
    this.subs = {};      // topic → [callback]
    this.opId = 1;
    this.connected = false;
    this.onStatusChange = null;
  }

  connect() {
    this.ws = new WebSocket(this.url);
    this.ws.onopen = () => {
      this.connected = true;
      this.onStatusChange?.('connected');
      // Re-subscribe if reconnecting
      Object.keys(this.subs).forEach((topic) => this._sendSubscribe(topic));
    };
    this.ws.onclose = () => {
      this.connected = false;
      this.onStatusChange?.('disconnected');
      setTimeout(() => this.connect(), 2500); // auto-reconnect
    };
    this.ws.onerror = () => {
      this.onStatusChange?.('error');
    };
    this.ws.onmessage = (evt) => {
      try {
        const msg = JSON.parse(evt.data);
        if (msg.op === 'publish') {
          (this.subs[msg.topic] || []).forEach((cb) => cb(msg.msg));
        }
      } catch (_) { /* ignore parse errors */ }
    };
  }

  _sendSubscribe(topic) {
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) return;
    this.ws.send(JSON.stringify({
      op: 'subscribe',
      id: `sub_${this.opId++}`,
      topic,
    }));
  }

  subscribe(topic, callback) {
    if (!this.subs[topic]) {
      this.subs[topic] = [];
      if (this.connected) this._sendSubscribe(topic);
    }
    this.subs[topic].push(callback);
    return () => {
      this.subs[topic] = this.subs[topic].filter((cb) => cb !== callback);
    };
  }

  publish(topic, type, msg) {
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) return;
    this.ws.send(JSON.stringify({ op: 'publish', topic, type, msg }));
  }

  disconnect() {
    this.ws?.close();
  }
}

// ── Singleton bridge shared across hook instances ───────────────────────────
let _bridge = null;
function getBridge() {
  if (!_bridge) {
    _bridge = new RosBridge(WS_URL);
    _bridge.connect();
  }
  return _bridge;
}

// ── Default Broadcaster / Dock Station Configuration ───────────────────────
export const DEFAULT_DOCK_CONFIG = {
  rosX: 5.0,
  rosY: 12.0,
  radiusMeters: 6.0,
};

// ── Main hook ───────────────────────────────────────────────────────────────
export function useRos() {
  const [rosStatus, setRosStatus] = useState('connecting'); // 'connecting'|'connected'|'disconnected'|'error'
  const [liveRobots, setLiveRobots]   = useState(null);   // null = not yet received
  const [liveTasks, setLiveTasks]     = useState(null);
  const [liveBundles, setLiveBundles] = useState({});      // robot_id → Bundle msg
  const [dockInfo, setDockInfo]       = useState(() => {
    const svgPos = rosToSvg(DEFAULT_DOCK_CONFIG.rosX, DEFAULT_DOCK_CONFIG.rosY);
    return {
      rosX: DEFAULT_DOCK_CONFIG.rosX,
      rosY: DEFAULT_DOCK_CONFIG.rosY,
      radiusMeters: DEFAULT_DOCK_CONFIG.radiusMeters,
      x: svgPos.x,
      y: svgPos.y,
      radiusPx: Math.round(DEFAULT_DOCK_CONFIG.radiusMeters / MAP_CONFIG.resolution),
    };
  });
  const robotIndexRef = useRef({});    // robot_id → palette index (stable)
  const taskMapRef    = useRef({});    // task_id  → merged task object

  // Stable colour index for robots
  const getRobotIndex = useCallback((robotId) => {
    if (!(robotId in robotIndexRef.current)) {
      robotIndexRef.current[robotId] = Object.keys(robotIndexRef.current).length;
    }
    return robotIndexRef.current[robotId];
  }, []);

  useEffect(() => {
    const bridge = getBridge();
    bridge.onStatusChange = setRosStatus;

    // ── /fleet/robot_states ──────────────────────────────────────────────
    const unsubRobots = bridge.subscribe('/fleet/robot_states', (msg) => {
      setLiveRobots((prev) => {
        const map = {};
        (prev || []).forEach((r) => { map[r.id] = r; });
        const svgPos = rosToSvg(
          msg.current_pose?.pose?.position?.x ?? 0,
          msg.current_pose?.pose?.position?.y ?? 0,
        );
        const id = msg.robot_id;
        const idx = getRobotIndex(id);
        const palette = robotColor(idx);
        const qz = msg.current_pose?.pose?.orientation?.z ?? 0;
        const qw = msg.current_pose?.pose?.orientation?.w ?? 1;
        const qx = msg.current_pose?.pose?.orientation?.x ?? 0;
        const qy = msg.current_pose?.pose?.orientation?.y ?? 0;
        const yaw = Math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz));
        const angleDeg = Math.round(-yaw * (180 / Math.PI));

        map[id] = {
          id,
          x: svgPos.x,
          y: svgPos.y,
          yaw,
          angleDeg,
          battery: Math.round(msg.battery_level ?? 100),
          online: true,
          state: msg.status === 0 ? 'idle' : 'active',
          task: msg.current_task_id || '—',
          color: palette.color,
          colorDim: palette.colorDim,
        };
        return Object.values(map);
      });
    });

    // ── /map (dynamic calibration if published) ──────────────────────────
    const unsubMap = bridge.subscribe('/map', (msg) => {
      if (msg?.info) {
        MAP_CONFIG.originX = msg.info.origin?.position?.x ?? MAP_CONFIG.originX;
        MAP_CONFIG.originY = msg.info.origin?.position?.y ?? MAP_CONFIG.originY;
        MAP_CONFIG.resolution = msg.info.resolution ?? MAP_CONFIG.resolution;
        MAP_CONFIG.width = msg.info.width ?? MAP_CONFIG.width;
        MAP_CONFIG.height = msg.info.height ?? MAP_CONFIG.height;
      }
    });

    // ── /fleet/task_pool (latched — initial seed) ────────────────────────
    const unsubPool = bridge.subscribe('/fleet/task_pool', (msg) => {
      (msg.tasks || []).forEach((t) => {
        if (!taskMapRef.current[t.task_id]) {
          taskMapRef.current[t.task_id] = t;
        }
      });
      _flushTasks(setLiveTasks, taskMapRef.current);
    });

    // ── /fleet/task_events (live state updates) ──────────────────────────
    const unsubEvents = bridge.subscribe('/fleet/task_events', (msg) => {
      const existing = taskMapRef.current[msg.task_id];
      // Monotonic guard: never downgrade state
      if (!existing || getTaskRank(msg.state) >= getTaskRank(existing.state)) {
        taskMapRef.current[msg.task_id] = msg;
        _flushTasks(setLiveTasks, taskMapRef.current);
      }
    });

    // ── /fleet/bundles ───────────────────────────────────────────────────
    const unsubBundles = bridge.subscribe('/fleet/bundles', (msg) => {
      setLiveBundles((prev) => ({ ...prev, [msg.robot_id]: msg }));
    });

    // ── /fleet/world_views (cross-check task states from each robot) ─────
    const unsubWV = bridge.subscribe('/fleet/world_views', (msg) => {
      let changed = false;
      (msg.task_states || []).forEach((t) => {
        const existing = taskMapRef.current[t.task_id];
        if (!existing || getTaskRank(t.state) >= getTaskRank(existing.state)) {
          taskMapRef.current[t.task_id] = t;
          changed = true;
        }
      });
      if (changed) _flushTasks(setLiveTasks, taskMapRef.current);
    });

    // ── /fleet/dock_pose (broadcaster location and radio range) ───────────
    const unsubDock = bridge.subscribe('/fleet/dock_pose', (msg) => {
      const rx = msg.pose?.position?.x ?? DEFAULT_DOCK_CONFIG.rosX;
      const ry = msg.pose?.position?.y ?? DEFAULT_DOCK_CONFIG.rosY;
      const svgPos = rosToSvg(rx, ry);
      setDockInfo({
        rosX: rx,
        rosY: ry,
        radiusMeters: DEFAULT_DOCK_CONFIG.radiusMeters,
        x: svgPos.x,
        y: svgPos.y,
        radiusPx: Math.round(DEFAULT_DOCK_CONFIG.radiusMeters / MAP_CONFIG.resolution),
      });
    });

    return () => {
      unsubRobots();
      unsubMap();
      unsubPool();
      unsubEvents();
      unsubBundles();
      unsubWV();
      unsubDock();
    };
  }, [getRobotIndex]);

  // ── Broadcast staged batch of tasks to /fleet/broadcast_tasks ─────────────
  const broadcastTasks = useCallback((stagedTasks) => {
    if (!stagedTasks || stagedTasks.length === 0) return false;
    const bridge = getBridge();
    if (!bridge.connected) {
      console.warn('Cannot broadcast: ROS bridge is not connected');
      return false;
    }

    const tasksPayload = stagedTasks.map((t, idx) => {
      const pRos = t.pickupRos || svgToRos(t.start.x, t.start.y);
      const dRos = t.dropoffRos || svgToRos(t.end.x, t.end.y);
      const taskId = t.name || `T${idx + 1}`;

      return {
        task_id: taskId,
        state: 0, // STATE_AVAILABLE
        assigned_robot_id: '',
        priority: t.priority ?? 1,
        version: 1,
        pickup_pose: {
          header: { frame_id: 'map', stamp: { sec: 0, nanosec: 0 } },
          pose: {
            position: { x: pRos.x, y: pRos.y, z: 0.0 },
            orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
          },
        },
        dropoff_pose: {
          header: { frame_id: 'map', stamp: { sec: 0, nanosec: 0 } },
          pose: {
            position: { x: dRos.x, y: dRos.y, z: 0.0 },
            orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
          },
        },
      };
    });

    bridge.publish('/fleet/broadcast_tasks', 'fleet_interfaces/msg/TaskPool', {
      header: { frame_id: 'map', stamp: { sec: 0, nanosec: 0 } },
      tasks: tasksPayload,
    });

    return true;
  }, []);

  return { rosStatus, liveRobots, liveTasks, liveBundles, dockInfo, broadcastTasks };
}

// ── Convert taskMap → UI task array ─────────────────────────────────────────
function _flushTasks(setter, taskMap) {
  const arr = Object.values(taskMap).map((t, index) => {
    const pickupX = t.pickup_pose?.pose?.position?.x ?? 0;
    const pickupY = t.pickup_pose?.pose?.position?.y ?? 0;
    const dropoffX = t.dropoff_pose?.pose?.position?.x ?? 0;
    const dropoffY = t.dropoff_pose?.pose?.position?.y ?? 0;
    const svgStart = rosToSvg(pickupX, pickupY);
    const svgEnd   = rosToSvg(dropoffX, dropoffY);
    return {
      number: index + 1,
      name: t.task_id,
      assigned: (t.state ?? 0) >= 1,
      robot: t.assigned_robot_id || null,
      progress: taskProgress(t.state ?? 0),
      state: t.state ?? 0,
      stateLabel: STATE_LABELS[t.state ?? 0] ?? 'Unknown',
      start: svgStart,
      end: svgEnd,
    };
  });
  setter(arr);
}
