#include <stdint.h>

#define NODES 32
#define DIRS 4
#define INVALID 255

#define DIR_UP    0
#define DIR_RIGHT 1
#define DIR_DOWN  2
#define DIR_LEFT  3

#define FACING_DIR DIR_UP

#define SWITCHES_BASE1 0xF0000000
#define SWITCHES_BASE2 0xF0000004

#define MOTOR_PORT (*(volatile uint8_t*)0xF0000010)
#define SENSOR_BASE (*(volatile uint8_t*)0xF0000020)
#define OBSTACLE_SENSOR_BASE (*(volatile uint8_t*)0xF0000030)

#define LEFT_MOTOR_FWD 0x01
#define RIGHT_MOTOR_FWD 0x02
#define LEFT_MOTOR_REV 0x04
#define RIGHT_MOTOR_REV 0x08
#define LEFT_MOTOR_STOP 0x00
#define RIGHT_MOTOR_STOP 0x00

#define SENSOR_LEFT_BIT   0
#define SENSOR_CENTER_BIT 1
#define SENSOR_RIGHT_BIT  2
#define SENSOR_ALL_WHITE 0b000 
#define SENSOR_LEFT_ONLY 0b001 
#define SENSOR_CENTER_ONLY 0b010 
#define SENSOR_RIGHT_ONLY 0b100 
#define SENSOR_LEFT_CENTER 0b011 
#define SENSOR_CENTER_RIGHT 0b110 
#define SENSOR_ALL_BLACK 0b111 
#define OBSTACLE_DETECT_BIT 0 

#define CMD_FORWARD 0
#define CMD_RIGHT   1
#define CMD_LEFT    2
#define CMD_AROUND  3


#define RAM_START_BASE 0x00011900

#define ARR_BASE RAM_START_BASE 

#define VISITED_BASE (ARR_BASE + (NODES * DIRS)) 
#define PARENT_BASE  (VISITED_BASE + sizeof(uint32_t)) 

#define QUEUE_BASE (PARENT_BASE + NODES) 
#define PATH_BASE QUEUE_BASE             

#define REAR_ADDR        (QUEUE_BASE + NODES) 
#define FRONT_ADDR       (REAR_ADDR + 1)      
#define PATH_LEN_ADDR    (FRONT_ADDR + 1) 
#define CURRENT_CMD_ADDR (PATH_LEN_ADDR + 1) 
#define INSTR_INDEX_ADDR (CURRENT_CMD_ADDR + 1) 
#define ORIENTATION_ADDR (INSTR_INDEX_ADDR + 1) 
#define START_NODE_ADDR  (ORIENTATION_ADDR + 1) 
#define TARGET_NODE_ADDR (START_NODE_ADDR + 1)  

#define INSTRUCTIONS_SIZE ((NODES - 1) * 2 / 4 + 1) 
#define COMMANDS_BASE (TARGET_NODE_ADDR + 1) 

#define MAX_PATH_NODES 9 
#define EXECUTED_PATH_BASE (COMMANDS_BASE + INSTRUCTIONS_SIZE) 
#define EXECUTED_PATH_IDX_ADDR (EXECUTED_PATH_BASE + MAX_PATH_NODES) 
#define BLOCKED_FROM_NODE_ADDR (EXECUTED_PATH_IDX_ADDR + 1) 
#define BLOCKED_TO_NODE_ADDR   (BLOCKED_FROM_NODE_ADDR + 1) 


#define arr ((volatile uint8_t (*)[4])ARR_BASE) 
#define visited (*(volatile uint32_t *)VISITED_BASE) 
#define parent ((volatile uint8_t *)PARENT_BASE)    
#define queue ((volatile uint8_t *)QUEUE_BASE)      
#define path ((volatile uint8_t *)PATH_BASE)        
#define front (*(volatile uint8_t *)FRONT_ADDR)     
#define rear (*(volatile uint8_t *)REAR_ADDR)       
#define path_len (*(volatile uint8_t *)PATH_LEN_ADDR) 
#define commands ((volatile uint8_t *)COMMANDS_BASE) 

#define current_cmd (*(volatile uint8_t *)CURRENT_CMD_ADDR) 
#define instr_index (*(volatile uint8_t *)INSTR_INDEX_ADDR) 
#define robot_orientation (*(volatile uint8_t *)ORIENTATION_ADDR) 
#define start (*(volatile uint8_t *)START_NODE_ADDR)         
#define target (*(volatile uint8_t *)TARGET_NODE_ADDR)       

#define executed_path ((volatile uint8_t *)EXECUTED_PATH_BASE)
#define executed_path_idx (*(volatile uint8_t *)EXECUTED_PATH_IDX_ADDR)
#define blocked_from_node (*(volatile uint8_t *)BLOCKED_FROM_NODE_ADDR)
#define blocked_to_node   (*(volatile uint8_t *)BLOCKED_TO_NODE_ADDR)

// UTILITY FUNCTIONS

// Reads a byte value from a given memory-mapped switch base address.
uint8_t get_switch_value(uint32_t base_addr) {
    return *(volatile uint8_t *)(base_addr);
}

// Simple busy-wait delay function.
#define DELAY_CYCLES 10000 // Standard delay amount for motor actions
void delay_cycles(unsigned int amount) {
    for (volatile unsigned int i = 0; i < amount; i++);
}

// Stops all robot motors.
void stop_robot() {
    MOTOR_PORT = 0;
}

// Checks if the robot's IR sensors indicate it is aligned on a line.
int alignment_ok() {
    uint8_t sensor_val = SENSOR_BASE;
    int ir_left   = (sensor_val >> SENSOR_LEFT_BIT) & 1;
    int ir_center = (sensor_val >> SENSOR_CENTER_BIT) & 1;
    int ir_right  = (sensor_val >> SENSOR_RIGHT_BIT) & 1;
    // Aligned if center sensor and at least one side sensor is detecting line
    return (ir_center == 1) && ((ir_left ==1) || (ir_right ==1));
}

// Corrects the robot's alignment to stay on the line.
void correct_alignment() {
    uint8_t sensor_val;
    while (!alignment_ok()) {
        sensor_val = SENSOR_BASE;
        int ir_left   = (sensor_val >> SENSOR_LEFT_BIT) & 1;
        int ir_center = (sensor_val >> SENSOR_CENTER_BIT) & 1;
        int ir_right  = (sensor_val >> SENSOR_RIGHT_BIT) & 1;

        if (ir_left && !ir_right) { // Robot drifting right, correct by turning left
            MOTOR_PORT = LEFT_MOTOR_REV | RIGHT_MOTOR_FWD;
        } else if (!ir_left && ir_right) { // Robot drifting left, correct by turning right
            MOTOR_PORT = LEFT_MOTOR_FWD | RIGHT_MOTOR_REV;
        } else { // Keep moving forward slightly if unsure but off-line
            MOTOR_PORT = LEFT_MOTOR_FWD | RIGHT_MOTOR_FWD;
        }
        delay_cycles(DELAY_CYCLES / 2);
        MOTOR_PORT = 0;
        delay_cycles(DELAY_CYCLES / 4);
    }
}

// Checks if the obstacle sensor detects an object.
int is_obstacle_detected() {
    return (*(volatile uint8_t*)OBSTACLE_SENSOR_BASE >> OBSTACLE_DETECT_BIT) & 1;
}

// Moves the robot backward for one grid unit.
void move_backward_one_grid() {
    MOTOR_PORT = LEFT_MOTOR_REV | RIGHT_MOTOR_REV;
    delay_cycles(DELAY_CYCLES * 3); // Same duration as move_forward_one_grid
    MOTOR_PORT = 0;
}

// MOTOR CONTROL FUNCTIONS

// Performs a turn based on the given command and updates robot's orientation.
void perform_turn(uint8_t cmd, volatile uint8_t* orientation) {
    switch (cmd) {
        case CMD_RIGHT:
            *orientation = (*orientation + 1) % 4; // 90 degrees clockwise
            MOTOR_PORT = LEFT_MOTOR_FWD | RIGHT_MOTOR_REV;
            delay_cycles(DELAY_CYCLES);
            break;
        case CMD_LEFT:
            *orientation = (*orientation + 3) % 4; // 90 degrees counter-clockwise
            MOTOR_PORT = LEFT_MOTOR_REV | RIGHT_MOTOR_FWD;
            delay_cycles(DELAY_CYCLES);
            break;
        case CMD_AROUND:
            *orientation = (*orientation + 2) % 4; // 180 degrees turn
            MOTOR_PORT = LEFT_MOTOR_FWD | RIGHT_MOTOR_REV;
            delay_cycles(2 * DELAY_CYCLES); // Double delay for 180 degree turn
            break;
        default:; 
    }
    MOTOR_PORT = 0; // Stop motors after turn
}

// Moves the robot forward for one grid unit.
void move_forward_one_grid() {
    MOTOR_PORT = LEFT_MOTOR_FWD | RIGHT_MOTOR_FWD;
    delay_cycles(DELAY_CYCLES * 3); // Adjust for specific grid size
    MOTOR_PORT = 0; // Stop motors after movement
}

// GRAPH AND PATHFINDING (BFS)

// Graph adjacency data: [UP, RIGHT, DOWN, LEFT] order for each node.
// Indices 2 and 3 are swapped for DOWN and LEFT to match corrected DIR_ macros.
const uint8_t initial_graph_data[NODES][DIRS] = {
    /* Node 00 */  {10, 1, 6, INVALID},      
    /* Node 01 */  {11, INVALID, 2, 0},
    /* Node 02 */  {1, 3, 4, 5},
    /* Node 03 */  {INVALID, INVALID, INVALID, 2},
    /* Node 04 */  {INVALID, INVALID, INVALID, 2}, 
    /* Node 05 */  {INVALID, 2, INVALID, INVALID},
    /* Node 06 */  {0, 7, 8, 9},             
    /* Node 07 */  {INVALID, INVALID, INVALID, 6},
    /* Node 08 */  {INVALID, 6, INVALID, INVALID}, 
    /* Node 09 */  {INVALID, 6, INVALID, INVALID},
    /* Node 10 */  {24, 11, 0, 26},
    /* Node 11 */  {19, 12, 1, 10},
    /* Node 12 */  {13, 14, INVALID, 11},
    /* Node 13 */  {INVALID, INVALID, 12, INVALID},
    /* Node 14 */  {16, INVALID, 12, 15},
    /* Node 15 */  {INVALID, 14, INVALID, INVALID},
    /* Node 16 */  {INVALID, 14, 17, 18},
    /* Node 17 */  {16, INVALID, INVALID, INVALID},
    /* Node 18 */  {INVALID, 16, 19, 21},
    /* Node 19 */  {18, INVALID, 11, 20},
    /* Node 20 */  {INVALID, 19, INVALID, INVALID},
    /* Node 21 */  {INVALID, 18, 22, 23},
    /* Node 22 */  {21, INVALID, INVALID, INVALID},
    /* Node 23 */  {INVALID, 21, 24, 30},
    /* Node 24 */  {23, 25, 10, INVALID},
    /* Node 25 */  {INVALID, INVALID, INVALID, 24},
    /* Node 26 */  {27, 10, INVALID, 28},
    /* Node 27 */  {INVALID, INVALID, 26, INVALID},
    /* Node 28 */  {30, 29, 26, INVALID},
    /* Node 29 */  {INVALID, INVALID, INVALID, 28},
    /* Node 30 */  {INVALID, 23, 31, 28},
    /* Node 31 */  {30, INVALID, INVALID, INVALID}
};

void init_graph() {
    for (int i = 0; i < NODES; i++) {
        for (int j = 0; j < DIRS; j++) {
            arr[i][j] = initial_graph_data[i][j];
        }
    }
}

void mark_visited(uint8_t node) { visited |= (1U << node); }

int is_visited(uint8_t node) { return (visited & (1U << node)) != 0; }

void enqueue(uint8_t node) {
    if (((rear + 1) % NODES) != front) { // Check for queue not full
        queue[rear] = node;
        rear = (rear + 1) % NODES;
    }
}

uint8_t dequeue() {
    if (front == rear) return INVALID; // Check for queue empty
    uint8_t node = queue[front];
    front = (front + 1) % NODES;
    return node;
}

void bfs(uint8_t start_node) {
    for (int i = 0; i < NODES; i++) parent[i] = INVALID; 
    visited = 0; 
    front = rear = 0; 
    enqueue(start_node);
    mark_visited(start_node);
    parent[start_node] = INVALID; 
    while (front != rear) { 
        uint8_t current = dequeue();
        for (uint8_t dir = 0; dir < DIRS; dir++) {
            uint8_t next = arr[current][dir]; 
            if (next != INVALID && next < NODES && !is_visited(next)) {
                if (current == blocked_from_node && next == blocked_to_node) {
                    continue; 
                }
                mark_visited(next);
                parent[next] = current; 
                enqueue(next);
            }
        }
    }
}

void store_path(uint8_t start_node, uint8_t target_node) {
    uint8_t temp_path[NODES]; 
    uint8_t len = 0;
    uint8_t current = target_node;
    while (1) {
        temp_path[len++] = current; 
        if (current == start_node) break; 
        current = parent[current]; 
        if (current == INVALID) { 
            path_len = 0; 
            return;
        }
    }
    uint8_t node_path_len = len; 
    for (uint8_t i = 0; i < node_path_len; i++) {
        path[i] = temp_path[node_path_len - 1 - i];
    }
    path_len = node_path_len; 
}

// COMMAND ENCODING AND DECODING

int get_direction(uint8_t from, uint8_t to) {
    for (unsigned int d = 0; d < DIRS; ++d) {
        if (arr[from][d] == to)
            return d;
    }
    return -1; 
}

uint8_t calc_command(int curr_dir, int next_dir) {
    int turn = (next_dir - curr_dir + 4) % 4; 
    switch (turn) {
        case 0: return CMD_FORWARD;  
        case 1: return CMD_RIGHT;    
        case 2: return CMD_AROUND;   
        case 3: return CMD_LEFT;     
        default: return CMD_FORWARD; 
    }
}

void encode_instruction(uint8_t* buf, unsigned int idx, uint8_t cmd) {
    unsigned int byte_idx = idx / 4; 
    unsigned int bit_offset = (idx % 4) * 2; 
    if (byte_idx < INSTRUCTIONS_SIZE) { 
        buf[byte_idx] &= ~(0x3 << bit_offset); 
        buf[byte_idx] |= ((cmd & 0x3) << bit_offset); 
    } else {
        
    }
}

uint8_t decode_instruction(const uint8_t* buf, unsigned int idx) {
    unsigned int byte_idx = idx / 4;
    unsigned int bit_offset = (idx % 4) * 2;
    return (buf[byte_idx] >> bit_offset) & 0x3; 
}

// INSTRUCTION GENERATION

void encode_path_to_instructions(uint8_t start_orientation_param) {
    if (path_len < 2) { 
        path_len = 0; 
        return;
    }

    int current_robot_orientation = start_orientation_param; 
    unsigned int actual_instr_count = 0; 

    for (unsigned int i = 0; i < path_len - 1; ++i) {
        int curr_node = path[i];
        int next_node = path[i + 1];

        int dir_to_next_node = get_direction(curr_node, next_node);
        if (dir_to_next_node < 0) { 
            actual_instr_count = 0; 
            break; 
        }

        uint8_t turn_cmd = calc_command(current_robot_orientation, dir_to_next_node);

        encode_instruction((uint8_t*)commands, actual_instr_count, turn_cmd);
        actual_instr_count++;
        
        if (turn_cmd == CMD_RIGHT) {
            current_robot_orientation = (current_robot_orientation + 1) % 4;
        } else if (turn_cmd == CMD_LEFT) {
            current_robot_orientation = (current_robot_orientation + 3) % 4;
        } else if (turn_cmd == CMD_AROUND) {
            current_robot_orientation = (current_robot_orientation + 2) % 4;
        }

        encode_instruction((uint8_t*)commands, actual_instr_count, CMD_FORWARD);
        actual_instr_count++;
        
        current_robot_orientation = dir_to_next_node; 

        int entry_dir_at_next_node = get_direction(next_node, curr_node); 

        if (entry_dir_at_next_node != INVALID) { 
            int desired_conceptual_orientation_at_next_node = (entry_dir_at_next_node + 2) % 4; 
            
            if (current_robot_orientation != desired_conceptual_orientation_at_next_node) {
                current_robot_orientation = desired_conceptual_orientation_at_next_node; 
            }
        }
    }
    path_len = (uint8_t)actual_instr_count; 
}


typedef enum {
    FSM_INIT,               
    FSM_COMPUTE_PATH,       
    FSM_PATH_FOUND,         
    FSM_READ_INSTR,         
    FSM_TURN,               
    FSM_MOVE_FORWARD,       
    FSM_REORIENT_AT_NODE,   
    FSM_OBSTACLE_DETECTED,  
    FSM_COMPLETE            
} RobotState;

void run_robot_fsm() {
    RobotState state = FSM_INIT;
    blocked_from_node = INVALID;
    blocked_to_node = INVALID;

    while (1) { 
        switch (state) {
            case FSM_INIT:
                init_graph(); 
                current_cmd = CMD_FORWARD; 
                instr_index = 0; 
                robot_orientation = FACING_DIR; 
                start = get_switch_value(SWITCHES_BASE1); 
                target = get_switch_value(SWITCHES_BASE2); 
                
                executed_path_idx = 0;

                state = FSM_COMPUTE_PATH; 
                break;

            case FSM_COMPUTE_PATH:
                bfs(start); 
                state = FSM_PATH_FOUND; 
                break;

            case FSM_PATH_FOUND:
                store_path(start, target); 
                if (path_len == 0) { 
                    state = FSM_COMPLETE; 
                } else {
                    encode_path_to_instructions(robot_orientation); 
                    instr_index = 0; 
                    
                    if (executed_path_idx == 0 || executed_path[executed_path_idx - 1] != start) {
                        executed_path[executed_path_idx++] = start;
                    }

                    state = FSM_READ_INSTR; 
                }
                break;

            case FSM_READ_INSTR:
                if (instr_index >= path_len) { 
                    state = FSM_COMPLETE; 
                    break;
                }
                
                if (is_obstacle_detected()) {
                    state = FSM_OBSTACLE_DETECTED;
                    break; 
                }
                
                current_cmd = decode_instruction((const uint8_t*)commands, instr_index); 
                instr_index++; 
                
                state = (current_cmd == CMD_FORWARD) ? FSM_MOVE_FORWARD : FSM_TURN;
                break;

            case FSM_TURN:
                perform_turn(current_cmd, &robot_orientation); 
                state = FSM_MOVE_FORWARD; 
                break;

            case FSM_MOVE_FORWARD:
                move_forward_one_grid();
                state = FSM_REORIENT_AT_NODE; 
                break;

            case FSM_REORIENT_AT_NODE: {
                uint8_t sensor_val = SENSOR_BASE;
                if (!alignment_ok() || (sensor_val & 0x07) != SENSOR_ALL_BLACK) {
                    correct_alignment(); 
                    break; 
                }

                stop_robot(); 

                uint8_t segment_idx_completed = (instr_index - 1) / 2; 
                uint8_t current_physical_node = path[segment_idx_completed + 1]; 
                uint8_t previous_node_in_path = path[segment_idx_completed]; 

                int entry_dir_at_current_node = get_direction(current_physical_node, previous_node_in_path);

                if (entry_dir_at_current_node != INVALID) { 
                    int desired_alignment_orientation = (entry_dir_at_current_node + 2) % 4; 
                    
                    if (robot_orientation != desired_alignment_orientation) {
                        uint8_t reorient_cmd = calc_command(robot_orientation, desired_alignment_orientation);

                        if (reorient_cmd != CMD_FORWARD) { 
                            perform_turn(reorient_cmd, &robot_orientation); 
                        }
                    }
                }
                
                if (executed_path_idx == 0 || executed_path[executed_path_idx - 1] != current_physical_node) {
                     executed_path[executed_path_idx++] = current_physical_node;
                }
                
                state = FSM_READ_INSTR;
                break;
            }

            case FSM_OBSTACLE_DETECTED: { 
                stop_robot(); 

                uint8_t segment_idx_blocked = instr_index / 2; 
                uint8_t last_safe_node = path[segment_idx_blocked]; 
                uint8_t blocked_segment_next_node = path[segment_idx_blocked + 1]; 

                blocked_from_node = last_safe_node;
                blocked_to_node = blocked_segment_next_node; 

                move_backward_one_grid(); 

                start = last_safe_node; 

                for (int i = 0; i < NODES; i++) parent[i] = INVALID;
                visited = 0;
                front = rear = 0;
                
                bfs(start); 

                store_path(start, target);
                if (path_len == 0) {
                    state = FSM_COMPLETE; 
                } else {
                    encode_path_to_instructions(robot_orientation); 
                    instr_index = 0; 
                    
                    blocked_from_node = INVALID;
                    blocked_to_node = INVALID;

                    state = FSM_READ_INSTR; 
                }
                break;
            }

            case FSM_COMPLETE:
                stop_robot();
                blocked_from_node = INVALID;
                blocked_to_node = INVALID;
                executed_path_idx = 0; 
                return;

            default:
                stop_robot();
                return;
        }
    }
}

// Main Function

int main() {
    run_robot_fsm();
    return 0;
}

void _start() {
    asm volatile("li sp, 0x12000"); 
    main();
}
