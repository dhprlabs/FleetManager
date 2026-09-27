# FleetManager Development Guide

## System Overview
Decentralized multi-robot fleet management system using ROS 2 (Humble). Implements:
- Max-Sum task allocation with CRDT-backed state sharing
- Conflict resolution combining ORCA (continuous) and PIBT (discrete)
- Traffic reservation system for single-lane aisle segments
- Pickup validation for task completion verification
- React + Vite frontend for visualization via rosbridge

## Key Directories
- `src/fleet_manager/fleet_manager/` - Core ROS 2 nodes (conflict_resolver, reservation_manager, etc.)
- `src/fleet_manager/launch/` - ROS 2 launch files for system startup
- `src/fleet_interfaces/` - Custom message/service definitions
- `src/virtual_lab/` - Gazebo simulation environment and worlds
- `frontend/` - React web interface connecting via rosbridge

## Essential Commands

### Build System
```bash
# Build all packages
colcon build --symlink-install

# Build specific package
colcon build --packages-select fleet_manager --symlink-install

# Source build environment
source install/setup.bash
```

### Running the System
```bash
# Launch full system with 3 robots (no auto-tasks)
ros2 launch fleet_manager fleet_manager.launch.py

# Launch with auto-generated sample tasks T1-T5
ros2 launch fleet_manager fleet_manager.launch.py auto_generate_sample_tasks:=true

# Adjust communication radius (meters) - critical for multi-robot discovery
ros2 launch fleet_manager fleet_manager.launch.py communication_radius:=15.0

# Launch without RViz visualizer (faster startup)
ros2 launch fleet_manager fleet_manager.launch.py launch_visualizer:=false
```

### Frontend Development
```bash
# Start frontend dev server (connects to rosbridge at ws://localhost:9090)
cd frontend
npm run dev

# Build for production
npm run build
```

### Testing
```bash
# Run all tests
pytest src/fleet_manager/test/

# Run specific test category
pytest src/fleet_manager/test/test_phase12_traffic_reservation.py -v

# Run linter (flake8)
flake8 src/fleet_manager/fleet_manager/
```

## Important Notes

### Communication Topology
- Robots discover peers via P2P beacons on UDP multicast
- Default communication radius: 6.0m (may need increase for larger maps)
- Task allocation requires robots to be within communication range
- Frontend connects via rosbridge_websocket (default port 9090)

### Coordinate System
- Map origin: [-5.807, -7.229, 0] meters
- Resolution: 0.05 m/pixel
- Frontend SVG coordinates map ROS meters via MAP_CONFIG in useRos.js

### Key Nodes (run per robot namespace)
1. `state_manager` - CRDT state sharing & gossiping
2. `task_manager` - Local task state tracking
3. `maxsum_allocator` - Event-driven Max-Sum task allocation
4. `bundle_manager` - Local task ordering & execution planning
5. `conflict_resolver` - ORCA + PIBT physical coordination
6. `reservation_manager` - Single-lane aisle reservations
7. `nav2_bridge` - Nav2 integration with traffic coordination
8. `pickup_validator` - Task completion verification
9. `p2p_transport` - Peer-to-peer communication layer

### Message Flow Highlights
- Task allocation: MaxSum → Bundle Manager → Nav2 Bridge
- Aisle access: Conflict Resolver ↔ Reservation Manager (via /fleet/reservations)
- Collision avoidance: Conflict Resolver overrides /cmd_vel when conflicts detected
- Task completion: Pickup Validator confirms item presence at pickup/dropoff

### Common Gotchas
1. **Communication Range**: Robots must be within `communication_radius` to discover peers and share state
2. **Namespace Isolation**: Each robot's nodes run in its own namespace (e.g., `/robot1/...`)
3. **Topic Topics**: 
   - `/traffic/reserve` and `/traffic/release` are namespaced per robot
   - `/fleet/reservations` is global (used for reservation coordination)
   - `/fleet/*` topics are global for fleet-wide state
4. **Simulation Timing**: Set `use_sim_time:=true` when using Gazebo
5. **Initial Spawn**: Robots spawn at default positions - ensure communication radius covers initial distances
6. **Frontend Connection**: Ensure rosbridge_websocket is running on port 9090 for frontend connectivity

## Architecture Boundaries
- **Fleet Manager Core**: `src/fleet_manager/fleet_manager/` - ROS 2 nodes implementing decentralized algorithms
- **Messages/Services**: `src/fleet_interfaces/` - Custom ROS 2 interfaces
- **Simulation**: `src/virtual_lab/` - Gazebo worlds, models, and launch files
- **Visualization**: `frontend/` - React app connecting to ROS via rosbridge

## Development Workflow
1. Modify source code in `src/fleet_manager/fleet_manager/`
2. Rebuild: `colcon build --packages-select fleet_manager --symlink-install`
3. Source environment: `source install/setup.bash`
4. Test changes via launch file or individual node execution
5. For frontend: modify `frontend/src/` and restart dev server


### Aisle Configuration
- Aisle definitions are loaded from `src/fleet_manager/config/aisle_segments.json`
- This file can be modified to change aisle positions and properties
- The system supports up to 4 single-lane aisles by default

### Rosbridge Connection
- Frontend connects to rosbridge_websocket at ws://localhost:9090
- Rosbridge must be running separately (not launched by fleet_manager.launch.py)
- To start rosbridge: `ros2 launch rosbridge_server rosbridge_websocket_launch.xml`
- Ensure port 9090 is available and accessible

### Test Structure
Tests are organized by development phase:
- `test_phase11_pickup_validation.py` - Pickup validation (Phase 11)
- `test_phase12_traffic_reservation.py` - Traffic reservation system (Phase 12)
- `test_phase13_conflict_resolution.py` - Conflict resolution (Phase 13)
- `test_phase14_dynamic_reallocation.py` - Dynamic task reallocation (Phase 14)
- `test_crdt.py` - CRDT functionality
- Plus copyright, linting, and pep257 tests
