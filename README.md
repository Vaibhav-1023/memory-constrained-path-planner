# Robot Navigation System (Embedded C)

This repository contains the embedded C code for an autonomous robot designed to navigate a predefined grid-based environment. The system implements a robust Finite State Machine (FSM) for control, leverages Breadth-First Search (BFS) for efficient path planning, and incorporates dynamic obstacle avoidance capabilities.

The primary goal of this project is to demonstrate a memory-efficient and reliable navigation solution for resource-constrained microcontroller environments.

## Features

* **Grid-Based Navigation:** The robot operates on a static, predefined 32-node grid map. Connections between nodes are cardinal (Up, Right, Down, Left) and defined in the `initial_graph_data` adjacency list.

* **Memory-Efficient Path Planning:** This system is highly optimized for minimal RAM usage. It employs a custom memory map and intelligent data overlapping techniques. The total RAM footprint for all dynamic data structures is meticulously managed to fit **under 256 bytes**. This is achieved by reusing memory blocks for data that is active during different, non-overlapping phases of the program's execution.

* **Breadth-First Search (BFS):** A standard BFS algorithm is implemented to discover the shortest path (in terms of node count) from a given start node to a target node. It utilizes `visited` and `parent` arrays to track search progress and reconstruct the optimal path.

* **Finite State Machine (FSM) Control:** A robust FSM (`run_robot_fsm`) dictates the robot's high-level behavior. States include initialization, path computation, instruction reading, motor control, alignment, and obstacle handling. This structured approach ensures predictable and manageable robot behavior.

* **Precise Movement Control:** Dedicated functions (`perform_turn`, `move_forward_one_grid`) translate high-level commands into low-level motor control signals via memory-mapped registers. Turns are precisely executed for 90-degree increments (Right, Left, Around) and forward movement covers one grid unit.

* **Line Following & Alignment:** The robot uses three IR sensors (`SENSOR_BASE`) to detect and follow lines. The `alignment_ok` and `correct_alignment` functions provide reactive control to keep the robot centered on the track, crucial for accurate node detection.

* **Dynamic Obstacle Avoidance:**

  * **Real-time Detection:** An additional IR sensor (`OBSTACLE_SENSOR_BASE`) continuously monitors for objects in the robot's immediate path.

  * **Immediate Reaction:** Upon obstacle detection, the robot immediately stops.

  * **Intelligent Backtracking:** The robot automatically backs up one full grid unit to clear the obstacle.

  * **Adaptive Re-planning:** The system identifies the specific edge that was blocked and dynamically re-runs the BFS algorithm. The BFS is modified to *exclude* this blocked edge from its search, forcing the calculation of an alternative optimal path.

  * **Seamless Resumption:** The robot then resumes navigation on the newly computed optimal path, effectively bypassing the obstacle.

* **Conceptual Path Re-orientation:** A unique logic within the `encode_path_to_instructions` function intelligently adjusts the robot's internal "belief" about its orientation at certain nodes (e.g., where the physical track implies a bend not captured by simple grid connections). This re-orientation is purely conceptual (updates an internal variable) and **does not encode an additional physical turn command**, ensuring subsequent turns are calculated correctly from the robot's desired conceptual facing.

## Crucial Code Logic Explained

### Direction Encoding

The system uses a consistent numerical mapping for cardinal directions, fundamental for all pathfinding and movement calculations:

* `DIR_UP = 0` (North)

* `DIR_RIGHT = 1` (East)

* `DIR_DOWN = 2` (South)

* `DIR_LEFT = 3` (West)

This sequential numerical assignment is crucial for the `calc_command` function, which relies on modulo arithmetic to determine relative turns. For instance, a turn from `DIR_UP` (0) to `DIR_RIGHT` (1) is a `+1` change, while a turn from `DIR_UP` (0) to `DIR_LEFT` (3) is a `+3` change (or `-1` equivalent), correctly representing 90-degree clockwise and 90-degree counter-clockwise rotations, respectively.

### Graph Encoding

The grid map is encoded as an **adjacency list** using a 2D array, `initial_graph_data[NODES][DIRS]`.

* Each row `initial_graph_data[node_ID]` represents a specific node in the grid.

* The columns within each row correspond to the four cardinal directions (`UP`, `RIGHT`, `DOWN`, `LEFT`), as defined by the `DIR_` macros.

* The value at `initial_graph_data[node_ID][direction]` is the ID of the neighboring node in that specific direction.

* `INVALID` (255) indicates that there is no connection (no path) from `node_ID` in that particular direction.

This format allows for quick lookup of neighbors and is directly used by the BFS algorithm.

### BFS Algorithm Explained

The Breadth-First Search (BFS) algorithm (`bfs` function) is used to find the shortest path between a starting node and all other reachable nodes in the grid.

1.  **Initialization:**

    * The `parent` array is cleared (`INVALID`) to track predecessors.

    * The `visited` bitmask is cleared to mark all nodes as unvisited.

    * The `queue` (implemented as a circular buffer with `front` and `rear` pointers) is initialized empty.

    * The `start_node` is enqueued and marked as visited.

2.  **Exploration:**

    * The algorithm repeatedly dequeues a `current` node.

    * For each of `current`'s neighbors (in all `DIRS`):

        * It checks if the `next` node is valid, within bounds, and unvisited.

        * **Obstacle Avoidance Integration:** Crucially, it checks if the edge from `current` to `next` is currently marked as `blocked_from_node` to `blocked_to_node`. If it is, that neighbor is skipped for the current pathfinding attempt.

        * If the `next` node is valid and not blocked, it's marked as visited, its `parent` is set to `current`, and it's enqueued.

3.  **Path Reconstruction (`store_path`):**

    * After BFS completes, `store_path` reconstructs the path from `target_node` back to `start_node` by following the `parent` links.

    * The path is built in reverse in a `temp_path` array and then copied to the global `path` array in the correct order.

    * `path_len` is set to the number of nodes in this computed path.

### Path Encoding

The `encode_path_to_instructions` function translates the high-level node sequence (from the `path` array) into a series of low-level, 2-bit movement commands (`CMD_FORWARD`, `CMD_RIGHT`, `CMD_LEFT`, `CMD_AROUND`) that the robot's FSM can directly execute. These commands are packed into the `commands` byte array.

1.  **Segment-by-Segment Translation:** The function iterates through each `(curr_node, next_node)` segment of the computed path.

2.  **Turn Command Generation:** For each segment, it calculates the `turn_cmd` required to orient the robot from its `current_robot_orientation` to face `next_node`. This `turn_cmd` is then encoded into the `commands` array.

3.  **Explicit Forward Command:** Immediately after encoding the `turn_cmd`, an explicit `CMD_FORWARD` command is encoded. This ensures that every logical step (turn + move) is represented by two distinct commands in the `commands` array, providing fine-grained control to the FSM.

4.  **Conceptual Re-orientation:** After encoding the commands for a segment, the `current_robot_orientation` (an internal variable used for calculation) is updated to reflect the robot's new physical orientation. Then, a crucial conceptual re-orientation check occurs:

    * It determines the direction the robot *entered* the `next_node` from (i.e., `next_node` back to `curr_node`).

    * It calculates the *opposite* of this entry direction as the `desired_conceptual_orientation`.

    * If the `current_robot_orientation` (its physical orientation after the move) is not equal to this `desired_conceptual_orientation`, the `current_robot_orientation` variable is updated to the `desired` value. **This update is purely internal and does NOT generate an additional physical turn command.** It ensures that the calculation for the *next* path segment's turn command is based on a logically consistent "straight ahead" orientation, especially for paths that might visually appear "curved" on the map.

5.  **`path_len` Update:** The global `path_len` variable is updated to store the total count of these 2-bit commands generated (e.g., 2 commands per segment).

### Object Detection

The dynamic obstacle avoidance system is integrated into the FSM to react to unexpected obstructions on the path.

1.  **Detection Mechanism:** A dedicated obstacle sensor (`OBSTACLE_SENSOR_BASE`) is polled in the `FSM_READ_INSTR` state. If `is_obstacle_detected()` returns true, the robot immediately transitions to `FSM_OBSTACLE_DETECTED`.

2.  **`FSM_OBSTACLE_DETECTED` State:**

    * The robot `stop_robot()` to prevent collision.

    * It identifies the specific edge (`blocked_from_node` to `blocked_to_node`) that was being traversed when the obstacle was detected. This edge is globally marked as blocked.

    * The robot performs a `move_backward_one_grid()` to physically clear the obstacle and return to the last safe node.

    * The `start` node for pathfinding is updated to this `last_safe_node`.

    * The `bfs` algorithm is then re-run. Since `bfs` is modified to skip the `blocked_from_node` to `blocked_to_node` edge, it will find an alternative path if one exists.

    * If a new path is found, it is encoded, the `blocked_from_node` and `blocked_to_node` flags are cleared, and the FSM resumes execution from `FSM_READ_INSTR` with the new path. If no alternative path is found, the robot transitions to `FSM_COMPLETE` (stuck).

## Memory Architecture Overview

The system employs a custom memory-mapped architecture to optimize RAM usage, a critical consideration for embedded systems. All key data structures are strategically placed and, in some cases, **overlay each other**, reusing memory for different phases of operation.

* **RAM Footprint:** The entire program's RAM usage is optimized to fit within **255 bytes**. This is achieved through:

    * **Overlapping `path` and `queue`:** The `path` array (storing the final node sequence after BFS) reuses the memory allocated for the `queue` (used only *during* BFS search). `path` is populated *after* `queue` is no longer actively needed for BFS.

    * **Overlapping `executed_path`/`blocked_nodes` with `visited`/`parent`:** The `executed_path` array (for obstacle avoidance history) and the `blocked_from_node`/`blocked_to_node` variables reuse the memory allocated for the `visited` bitmask and `parent` array. These BFS-specific arrays are only active during path computation and are no longer needed during path execution or obstacle handling.

    * **Minimal `INSTRUCTIONS_SIZE`:** The `commands` array size is precisely calculated to hold only the necessary encoded instructions (2 commands per path segment), rather than a larger, potentially wasteful buffer.

### Memory Usage Breakdown (RAM)

The following table details the RAM allocation for each major data structure and variable, providing a byte-by-byte breakdown of the program's memory footprint. The memory addresses and sizes confirm the adherence to the 256-byte RAM constraint.

| Variable/Structure | Base Address (Example) | Size (Bytes) | Purpose | Overlaps With | 
 | ----- | ----- | ----- | ----- | ----- | 
| `arr` | `0x00011900` | 128 | Adjacency list for graph connections (`uint8_t[NODES][DIRS]`) | None | 
| `visited` | `0x00011980` | 4 | BFS visited node bitmask (`uint32_t`) | `executed_path_idx`, `blocked_from_node`, `blocked_to_node` | 
| `parent` | `0x00011984` | 32 | BFS parent array for path reconstruction (`uint8_t[NODES]`) | `executed_path` | 
| `queue` | `0x000119A4` | 32 | BFS queue data (`uint8_t[NODES]`) | `path` | 
| `path` | `0x000119A4` | 32 | Computed node path (node sequence) (`uint8_t[NODES]`) | `queue` | 
| `rear` | `0x000119C4` | 1 | Queue rear pointer for BFS (`uint8_t`) | None | 
| `front` | `0x000119C5` | 1 | Queue front pointer for BFS (`uint8_t`) | None | 
| `path_len` | `0x000119C6` | 1 | Total number of encoded commands (`uint8_t`) | None | 
| `current_cmd` | `0x000119C7` | 1 | Command currently being processed by FSM (`uint8_t`) | None | 
| `instr_index` | `0x000119C8` | 1 | Index of the next command to read (`uint8_t`) | None | 
| `robot_orientation` | `0x000119C9` | 1 | Robot's current physical orientation (`uint8_t`) | None | 
| `start` | `0x000119CA` | 1 | Mission start node ID (`uint8_t`) | None | 
| `target` | `0x000119CB` | 1 | Mission target node ID (`uint8_t`) | None | 
| `commands` | `0x000119CC` | 16 | Encoded movement commands (`uint8_t[INSTRUCTIONS_SIZE]`) | None | 
| `executed_path` | `0x000119DC` | 9 | History of visited nodes for backtracking (`uint8_t[MAX_PATH_NODES]`) | None | 
| `executed_path_idx` | `0x000119FC` | 1 | Index/count for `executed_path` (`uint8_t`) | `visited` (partially) | 
| `blocked_from_node` | `0x000119FD` | 1 | Origin node ID of the blocked edge (`uint8_t`) | `visited` (partially) | 
| `blocked_to_node` | `0x000119FE` | 1 | Destination node ID of the blocked edge (`uint8_t`) | `visited` (partially) | 
| **Total** |  | **232** | **Total RAM Usage (bytes)** |  | 

*Note: Total RAM usage is calculated from the lowest (`0x00011900`) to the highest (`0x000119FE`) address used, accounting for all allocated space.*

*Note:* Total RAM usage is calculated from the lowest (`0x00011900`) *to the highest (`0x000119E7`) address used, accounting for all allocated space.*

## How to Use / Simulate

### Prerequisites

* **RISC-V GCC Toolchain:** Required to compile the C code for a RISC-V target.

* **RIPES Simulator:** Recommended for simulating the microcontroller environment, including memory-mapped I/O, sensors, and program execution.

### Compilation

1.  Save the provided C code as a `.c` file (e.g., `robot_navigator.c`).

2.  Compile using your RISC-V GCC toolchain. Example command (adjust as needed for your specific setup):

    ```bash
    riscv-none-embed-gcc -Oz -march=rv32i -mabi=ilp32 -nostdlib -o robot_navigator.elf robot_navigator.c
    ```

    *(Ensure `-nostdlib` is used as you're providing your own `_start` and not linking against standard libraries.)*

### Running in RIPES

1.  Open the RIPES simulator.

2.  Load the compiled `.elf` file (`robot_navigator.elf`).

3.  **Configure Memory-Mapped I/O:**

    * **Switches:** Set the values for `SWITCHES_BASE1` (`0xF0000000`) and `SWITCHES_BASE2` (`0xF0000004`) to your desired start and target node IDs.

    * **IR Sensors:** Simulate line sensor input at `SENSOR_BASE` (`0xF0000020`) using bitmasks (e.g., `0b111` for all on line).

    * **Obstacle Sensor:** Simulate obstacle detection at `OBSTACLE_SENSOR_BASE` (`0xF0000030`) by setting/clearing bit 0 (e.g., `0b01` for detected, `0b00` for clear).

4.  **Open Memory Viewer:** Monitor the RAM region starting from `0x00011900` to `0x000119FE` to observe variable states.

5.  **Run Simulation:** Observe the robot's simulated behavior and memory changes.

## Code Structure (High-Level)

The C code is organized into logical sections:

* **Global Definitions and Memory Map:** Defines all constants, memory addresses, and variable access macros.

* **Utility Functions:** Helper functions for hardware interaction (switches, delays, sensors) and basic robot movements.

* **Motor Control Functions:** Specific functions to control robot turns and forward movement.

* **Graph and Pathfinding (BFS):** Contains the `initial_graph_data`, `init_graph`, `bfs`, and `store_path` functions.

* **Command Encoding and Decoding:** Functions to convert high-level path segments into low-level 2-bit commands and vice-versa.

* **Instruction Generation:** The `encode_path_to_instructions` function, which translates the BFS path into the sequence of commands the robot will execute, incorporating conceptual re-orientation.

* **Robot FSM (Finite State Machine):** The `run_robot_fsm` function, which is the main control loop, managing state transitions and orchestrating all robot actions.

* **Main Function:** Entry point of the program.

## Detailed Documentation

For an in-depth explanation of the algorithms, detailed memory layout, specific optimization techniques, and step-by-step walkthroughs of the robot's behavior, please refer to the accompanying **[RISCV_microcontroller.pdf]**.
