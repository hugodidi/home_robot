# FSM Patrol Architecture (YASMIN)

The Advanced Patrol system relies on **[YASMIN](https://github.com/uleroboticsgroup/yasmin)** (Yet Another State MachINe) to decouple waypoint navigation from base sequential scripting. By modeling the patrol logic as a Finite State Machine (FSM), the robot can gracefully handle physical blockages, network errors, and asynchronous actions securely.

This module is contained entirely within the independent `home_robot_fsm` ROS 2 package so that the core navigation stack remains untouched.

---

## 🧭 General Flow

The machine operates cyclically, acquiring targets from a global pre-loaded queue residing in the `Blackboard` (a shared memory object). The basic execution graph contains the following core states:

### 1. `SELECT_WAYPOINT`
- **Role**: Memory manager.
- **Function**: Pops the first available waypoint from the patrol sequence queue from the Blackboard. It parses the coordinates $(x, y, \theta)$ and actively loads them into the current target cache. Once completed, it dictates a transition to begin navigation.
- **Outcomes**: `navigate`, `finished` (if the queue is completely empty).

### 2. `NAVIGATE`
- **Role**: Action Server wrapper.
- **Function**: This state inherently inherits from `yasmin_ros.ActionState`. It builds a `NavigateToPose.Goal` using the newly fetched coordinates from the Blackboard and relays them asynchronously to the Nav2 action server. Throughout the journey, it reads `.Feedback` packets to print the remaining distance without blocking execution.
- **Outcomes**: `succeeded`, `abort`, `cancel`.

### 3. `WAYPOINT_REACHED`
- **Role**: Inter-process synchronization.
- **Function**: Executed only upon a successful `/navigate_to_pose` arrival. It triggers a ROS 2 `String` message (`llegada:{waypoint}`) through the `/patrol_events` topic. If the `voice_controller` node is active, this string prompts the robot's Text-To-Speech engine to acoustically announce its arrival. 
- **Note**: It actively invokes a technical suspension of 1.5 seconds (`time.sleep`) to allow external visualizing networks (YASMIN viewer) enough window span to catch the state lighting up visually since it is technically a computationally instantaneous phase. 

### 4. `HANDLE_ERROR`
- **Role**: Emergency exit routing.
- **Function**: Dictates the fallback route when Nav2 natively aborts the pathfinding (no valid costmap route found). Based on CLI parameters (`--skip-errors`), it either tosses the corrupted waypoint back into the void and pulls the next one cleanly (`recover`), or acts defensively shutting down the whole FSM indefinitely out of caution (`abort`).

---

## 🚧 Active Anti-Stuck System

Unlike basic FSM algorithms that blindly rely on Nav2's native recovery behaviors exclusively, this FSM incorporates an overarching active physical watcher inside `NAVIGATE`.

### Problem
When the robot collides against a tough geometrical obstacle blindspot (or when pushing a lightweight piece of physical furniture), Nav2 might continuously compute paths dynamically believing the robot is just moving slowly, never actually throwing an `ABORT`. 

### The Solution: `CHECK_STUCK` & `STUCK_RECOVERY`
Within the `NAVIGATE` feedback loop, the module watches the distance deltas. If the distance to the goal stalls strictly within a physical threshold boundary ($\leq 0.1$ meters) for **over 15 contiguous seconds**, the node throws a `is_stuck` flag globally and actively kills its own action client sending the state down the `CANCEL` route.

1. **`CHECK_STUCK`**: A filter state catching any `CANCEL`. It checks the state of `is_stuck`. If the cancel was legitimate (User pressed Ctrl+C), it funnels to error. If it evaluates the flag to be True, it triggers the escape mechanism.
2. **`STUCK_RECOVERY`**: A pure manual override state. Evading Nav2 and global costmaps altogether, it directly captures control over the raw `/cmd_vel` multiplexer. It publishes a backward linear velocity of `-1.0 m/s` for approx 1.2 seconds to un-wedge the physical chassis geometries. 
3. After stopping the chassis smoothly, it transitions the machine **back into `NAVIGATE`**, aggressively restarting the Action Goal algorithm to bypass the localized space block relying purely on the new un-wedged Odometry pose.
