# Patrol Planners

This project provides three patrol planners over the same waypoint file:

```bash
ros2_ws/src/home_robot/config/waypoints.yaml
```

Assume all commands are run inside the Docker container from:

```bash
/home/ubuntu/home_robot
```

Start Nav2 first and keep it running:

```bash
./scripts/launch_navigation.sh --slam
```

Run only one patrol method at a time.

## Visualizers

- RViz/noVNC desktop: `http://localhost:6080`
- YASMIN Viewer: `http://localhost:5000`
- Groot2 live BT server: `localhost:1667`
- PDDL plan viewer: `http://localhost:8080`

Groot2 is installed by the Dockerfile via `scripts/install_groot2.sh`. If it is missing after pulling changes, rebuild Docker:

```bash
cd docker
docker compose up --build
```

## 1. YASMIN FSM

Explicit finite state machine patrol with stuck detection and recovery.

```bash
./scripts/launch_fsm_patrol.sh
```

Options:

```bash
./scripts/launch_fsm_patrol.sh --random
./scripts/launch_fsm_patrol.sh --skip-errors
```

Visualizer:

- `http://localhost:5000`
- RViz for robot/map/path
- `/patrol_events` for arrival messages

Stuck behavior: if the robot does not progress for 15 s, navigation is cancelled, the robot backs up for ~1 s, and the FSM retries the same waypoint.

## 2. BehaviorTree.CPP

Custom BT patrol implemented with BehaviorTree.CPP. Nav2 is only used as the `NavigateToPose` backend. Includes fallback recovery when navigation stalls.

```bash
./scripts/launch_bt_patrol.sh
```

Options:

```bash
./scripts/launch_bt_patrol.sh --random
./scripts/launch_bt_patrol.sh --skip-errors
./scripts/launch_bt_patrol.sh --no-groot
```

Tree file:

```bash
ros2_ws/src/home_robot_bt/config/bt_patrol.xml
```

Visualizer:

- Groot2 live monitor on port `1667`
- RViz for robot/map/path
- logs for current BT node and Nav2 action state

Stuck behavior: if distance remaining does not improve for 15 s, the BT aborts the current navigation branch, runs `BackUpRecovery` (reverse ~1 s), and retries the waypoint.

## 3. PDDL (POPF)

Formal PDDL planner using POPF directly with execution monitoring and replanning.

```text
waypoints.yaml -> PDDL problem -> POPF -> parse plan -> execute first action -> repeat
```

The executor maintains the set of already-visited waypoints across replans. If Nav2 fails or the robot is stuck, it backs up for ~1 s and regenerates the problem with the updated state before calling POPF again.

Run the planner:

```bash
./scripts/launch_pddl_patrol.sh
```

Options:

```bash
./scripts/launch_pddl_patrol.sh --random
./scripts/launch_pddl_patrol.sh --skip-errors
```

Domain:

```bash
ros2_ws/src/home_robot_pddl/pddl/patrol_domain.pddl
```

Initial symbolic pose:

- Uses TF `map -> base_link` to determine the nearest waypoint as the initial `(robot_at robot1 <waypoint>)`.

POPF-only check, without moving the robot:

```bash
./scripts/run_pddl_patrol.sh
```

This only prints the plan. It does not execute Nav2 actions.

Visualizer:

- RViz for robot/map/path
- `/pddl_patrol/markers` for waypoint markers
- PDDL plan viewer: `http://localhost:8080` (auto-refreshes every 500 ms)
- logs for symbolic plan execution

Stuck behavior: if a Nav2 action fails, the executor commands a 1-second reverse, then replans from the recovered position using POPF with the updated `visited` state.

To start only the plan viewer (useful for observing an already-running patrol):

```bash
ros2 run home_robot_pddl pddl_ui
```

## Comparison

| Method | Logic | Planner | Stuck recovery | Main viewer |
|---|---|---|---|---|
| YASMIN FSM | Explicit FSM states | Manual transitions | Back up + retry | YASMIN Viewer |
| BehaviorTree.CPP | Custom BT tree | BT fallback/recovery | Back up + retry | Groot2 |
| PDDL (POPF) | PDDL symbolic planning | POPF replanning | Back up + replan | RViz + logs |

## Typical Demo

Terminal 1:

```bash
./scripts/launch_navigation.sh --slam
```

Terminal 2, choose one:

```bash
./scripts/launch_fsm_patrol.sh
./scripts/launch_bt_patrol.sh
./scripts/launch_pddl_patrol.sh
```
