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
 *   /fleet/task_ownership (fleet_interfaces/msg/TaskOwnership)
 *   /fleet/decision_log  (std_msgs/msg/String containing allocator audit JSON)
 *   /fleet/traffic_events (std_msgs/msg/String containing reservation/ORCA/PIBT audit JSON)
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

// ── Default Single-Lane Aisle Segments ──────────────────────────────────────
export const DEFAULT_AISLES = [
  { id: 'aisle_1', segment_id: 'aisle_1', name: 'Aisle 1', x_min: 1.5, x_max: 4.0, y_min: 0.0, y_max: 2.5, is_single_lane: true },
  { id: 'aisle_2', segment_id: 'aisle_2', name: 'Aisle 2', x_min: 1.5, x_max: 4.0, y_min: -3.5, y_max: -1.0, is_single_lane: true },
  { id: 'aisle_3', segment_id: 'aisle_3', name: 'Aisle 3', x_min: -4.5, x_max: -2.0, y_min: -2.0, y_max: 2.0, is_single_lane: true },
];

export function normalizeAisles(rawAisles) {
  if (!Array.isArray(rawAisles)) return [];
  return rawAisles.map((a, idx) => {
    const segId = a.segment_id || a.id || `aisle_${idx + 1}`;
    const x1 = Number(a.x_min);
    const x2 = Number(a.x_max);
    const y1 = Number(a.y_min);
    const y2 = Number(a.y_max);
    return {
      id: segId,
      segment_id: segId,
      name: a.name || segId,
      x_min: Math.round(Math.min(x1, x2) * 1000) / 1000,
      x_max: Math.round(Math.max(x1, x2) * 1000) / 1000,
      y_min: Math.round(Math.min(y1, y2) * 1000) / 1000,
      y_max: Math.round(Math.max(y1, y2) * 1000) / 1000,
      is_single_lane: a.is_single_lane ?? true,
    };
  });
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
      } catch { /* ignore parse errors */ }
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

// ── Default Robot Spawn / Dock Positions ───────────────────────────────────
export const DEFAULT_SPAWN_POSES = {
  robot1:  { x: 7.7, y: 14.4 },
  robot2:  { x: 3.4, y: 14.6 },
  robot3:  { x: 4.3, y: 1.2 },
  robot_1: { x: 7.7, y: 14.4 },
  robot_2: { x: 3.4, y: 14.6 },
  robot_3: { x: 4.3, y: 1.2 },
};

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
  const [allocationEvents, setAllocationEvents] = useState([]);
  const [trafficEvents, setTrafficEvents] = useState([]);
  // robot_id → array of {x,y} SVG waypoints from Nav2's /plan topic
  const [navPaths, setNavPaths] = useState({});            // robot_id → [{x,y}]
  // segment_id → {holder, state} from /fleet/reservations
  const [liveReservations, setLiveReservations] = useState({});
  const subscribedPlanTopicsRef = useRef(new Set());       // track which plan topics we've subscribed
  const initialPosesRef = useRef({});

  // Per-robot dock stations: initialized to each robot's initial/spawn pose
  const [dockStations, setDockStations] = useState(() => {
    const initial = {};
    Object.entries(DEFAULT_SPAWN_POSES).forEach(([rid, pos]) => {
      const svg = rosToSvg(pos.x, pos.y);
      initial[rid] = { x: pos.x, y: pos.y, svgX: svg.x, svgY: svg.y };
    });
    return initial;
  });
  const [liveAisles, setLiveAisles]       = useState(() => {
    try {
      const saved = localStorage.getItem('fleet_aisle_config');
      if (saved) {
        const parsed = JSON.parse(saved);
        if (Array.isArray(parsed) && parsed.length > 0) {
          return normalizeAisles(parsed);
        }
      }
    } catch (_) {}
    return DEFAULT_AISLES;
  });
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

        const rx = msg.current_pose?.pose?.position?.x;
        const ry = msg.current_pose?.pose?.position?.y;
        if (rx !== undefined && ry !== undefined && !initialPosesRef.current[id]) {
          initialPosesRef.current[id] = { x: rx, y: ry };
          const initSvg = rosToSvg(rx, ry);
          setDockStations((prev) => ({
            ...prev,
            [id]: { x: rx, y: ry, svgX: initSvg.x, svgY: initSvg.y },
          }));
        }

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

    // ── /fleet/task_ownership (show ownership as soon as it is claimed) ──
    const unsubOwnership = bridge.subscribe('/fleet/task_ownership', (msg) => {
      // STATUS_CLAIMED and STATUS_CONFIRMED are 0 and 1 respectively.
      if (msg.status !== 0 && msg.status !== 1) return;
      const existing = taskMapRef.current[msg.task_id];
      if (!existing) return;
      taskMapRef.current[msg.task_id] = {
        ...existing,
        assigned_robot_id: msg.robot_id,
        // Preserve later lifecycle states received from task events.
        state: Math.max(existing.state ?? 0, 1),
      };
      _flushTasks(setLiveTasks, taskMapRef.current);
    });

    // ── /fleet/bundles ───────────────────────────────────────────────────
    const unsubBundles = bridge.subscribe('/fleet/bundles', (msg) => {
      setLiveBundles((prev) => ({ ...prev, [msg.robot_id]: msg }));
    });

    // ── /fleet/decision_log (recent Binary Max-Sum decisions) ───────────
    const unsubDecisionLog = bridge.subscribe('/fleet/decision_log', (msg) => {
      try {
        const event = JSON.parse(msg.data);
        setAllocationEvents((previous) => [...previous, event].slice(-12));
      } catch {
        // A malformed audit line must not interrupt the live dashboard.
      }
    });

    // ── /fleet/traffic_events (aisle reservations, ORCA, PIBT) ──────────
    const unsubTrafficEvents = bridge.subscribe('/fleet/traffic_events', (msg) => {
      try {
        const event = JSON.parse(msg.data);
        setTrafficEvents((previous) => [...previous, event].slice(-50));
      } catch {
        // Keep the dashboard usable if an individual diagnostic message is malformed.
      }
    });

    // ── /fleet/reservations (live aisle reservation grants) ──────────────
    const unsubReservations = bridge.subscribe('/fleet/reservations', (msg) => {
      // Reservation states: 0=requested,1=granted,2=active,3=released,4=waiting,5=expired,6=denied
      const LIVE_STATES = new Set([1, 2]); // granted | active
      const DONE_STATES = new Set([3, 5]); // released | expired
      setLiveReservations((prev) => {
        const next = { ...prev };
        const segId = msg.segment_id;
        if (!segId) return prev;
        if (LIVE_STATES.has(msg.state)) {
          next[segId] = { holder: msg.robot_id, state: msg.state };
        } else if (DONE_STATES.has(msg.state)) {
          if (next[segId]?.holder === msg.robot_id) delete next[segId];
        }
        return next;
      });
    });

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

    // ── /fleet/aisle_config (dynamic single-lane aisle segment definitions) ─
    const unsubAisles = bridge.subscribe('/fleet/aisle_config', (msg) => {
      try {
        const payload = typeof msg.data === 'string' ? JSON.parse(msg.data) : msg.data;
        const list = payload.aisles || payload;
        if (Array.isArray(list) && list.length > 0) {
          const normalized = normalizeAisles(list);
          setLiveAisles(normalized);
          try {
            localStorage.setItem('fleet_aisle_config', JSON.stringify(normalized));
          } catch (_) {}
        }
      } catch (err) {
        console.warn('Failed to parse /fleet/aisle_config:', err);
      }
    });

    return () => {
      unsubRobots();
      unsubMap();
      unsubPool();
      unsubEvents();
      unsubOwnership();
      unsubBundles();
      unsubDecisionLog();
      unsubTrafficEvents();
      unsubReservations();
      unsubWV();
      unsubDock();
      unsubAisles();
    };
  }, [getRobotIndex]);

  // ── Subscribe to Nav2 plan topics when new robots appear ─────────────────
  // Nav2 publishes the planned global path on /{robot_id}/plan (nav_msgs/Path).
  // We subscribe dynamically each time liveRobots changes so we don't miss
  // robots that come online after the initial mount.
  useEffect(() => {
    if (!liveRobots) return;
    const bridge = getBridge();
    liveRobots.forEach(({ id }) => {
      const topic = `/${id}/plan`;
      if (subscribedPlanTopicsRef.current.has(topic)) return;
      subscribedPlanTopicsRef.current.add(topic);
      bridge.subscribe(topic, (msg) => {
        // msg is nav_msgs/Path: { header, poses: [{header, pose}] }
        const poses = msg.poses || [];
        const svgPoints = poses
          .filter((_, i) => i % 3 === 0)          // subsample every 3rd point for perf
          .map((p) => rosToSvg(
            p.pose?.position?.x ?? 0,
            p.pose?.position?.y ?? 0,
          ));
        setNavPaths((prev) => ({ ...prev, [id]: svgPoints }));
      });
    });
  }, [liveRobots]);

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

  // ── Broadcast and save updated single-lane aisle configurations ─────────────
  const saveAislesConfig = useCallback((newAisles) => {
    const normalized = normalizeAisles(newAisles);
    setLiveAisles(normalized);
    try {
      localStorage.setItem('fleet_aisle_config', JSON.stringify(normalized));
    } catch (_) {}

    const bridge = getBridge();
    if (bridge && bridge.connected) {
      bridge.publish('/fleet/aisle_config', 'std_msgs/msg/String', {
        data: JSON.stringify({
          source_robot_id: 'frontend_ui',
          timestamp: Date.now() / 1000,
          aisles: normalized,
        }),
      });
    }
    return true;
  }, []);

  return {
    rosStatus,
    liveRobots,
    liveTasks,
    liveBundles,
    allocationEvents,
    trafficEvents,
    navPaths,
    liveReservations,
    dockStations,
    liveAisles,
    dockInfo,
    broadcastTasks,
    saveAislesConfig,
  };
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
