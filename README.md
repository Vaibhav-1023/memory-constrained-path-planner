# Bare-Metal BFS with Fixed Memory Mapping

A minimal Breadth-First Search (BFS) implementation for a 32-node graph, using explicit memory addresses for all data structures. Designed for low-level, bare-metal environments (such as Ripes or embedded hardware) with strict memory constraints and no standard C library dependencies.

## Introduction:
This project demonstrates how to implement BFS for shortest path discovery in a graph, storing all arrays and state variables at fixed memory addresses. This approach is useful for educational purposes, hardware simulation, or environments where linker scripts and dynamic memory are unavailable.

## Installation & Usage
1. **Clone the repository:**
2. **Build the project:**
- Use a RISC-V cross-compiler (e.g., `riscv64-unknown-elf-gcc`). We used [xPack GNU Compiler](https://github.com/xpack-dev-tools/riscv-none-elf-gcc-xpack/releases/tag/v12.3.0-1/) to compile the code using the following compiler flags in terminal.
  ```
  riscv64-unknown-elf-gcc -O0 -g -nostartfiles -ffreestanding -o bfs.elf bfs.c
  ```
3. **Run in Ripes:**
- Load `bfs.elf` and inspect memory at the following addresses:
  - `0x000110E0` : Path array (sequence of node indices)
  - `0x00011102` : Path length

## How it works:
- **Graph Representation:**  
The graph is stored at `0x00011000` as a 32x4 array, where each node has up to 4 neighbors (or 255 for no connection).
- **BFS State:**  
Arrays for visited nodes, parent tracking, and the BFS queue are mapped to fixed memory locations. 
- **Path Storage:**  
After BFS, the shortest path from node 8 to node 17 is reconstructed and stored at `0x000110E0`.
- **No Standard Library:**  
All memory operations are done manually, with no `memcpy` or standard C library calls.

## Brief working of BFS algorithm:
- Start at the source node: Mark it as visited and add it to the queue.
- Process the queue:
  - Remove the first node from the queue.
  -   Visit all its unvisited neighbors, mark them as visited, and add them to the end of the queue.
- Repeat: Continue the process until the queue is empty.

## Code Overview:
- `bfs.c` contains:
  - Memory-mapped arrays for graph, visited, parent, queue, and path.
  - Manual graph initialization.
  - Circular queue for BFS.
  - Path reconstruction logic.
  - Stack pointer initialization for bare-metal operation.

## Code Walkthrough:
### 1. Memory Mapping and Constants
  ```C
#define NODES 32
#define DIRS 4
#define INVALID 255
#define START 8
#define TARGET 20

// Fixed memory mapping
#define ARR_BASE        0x00011000
#define VISITED_BASE    0x00011080
#define PARENT_BASE     0x000110A0
#define QUEUE_BASE      0x000110C0
#define PATH_BASE       0x000110E0
#define FRONT_ADDR      0x000110EA
#define REAR_ADDR       0x000110E9
#define PATH_LEN_ADDR   0x000110EB

#define arr         ((volatile uint8_t (*)[4])ARR_BASE)
#define visited     ((volatile uint8_t *)VISITED_BASE)
#define parent      ((volatile uint8_t *)PARENT_BASE)
#define queue       ((volatile uint8_t *)QUEUE_BASE)
#define path        ((volatile uint8_t *)PATH_BASE)
#define front       (*(volatile uint8_t *)FRONT_ADDR)
#define rear        (*(volatile uint8_t *)REAR_ADDR)
#define path_len    (*(volatile uint8_t *)PATH_LEN_ADDR)
  ```
- NODES, DIRS, INVALID: Define the size of the graph (32 nodes), directions per node (4), and the value for an invalid connection (255).
- START, TARGET: The starting node and the target node for the BFS. We will later use UART communication to get these values from the memory itself.
- Memory Addresses: Each array and variable is mapped to a specific memory address.
- Pointer Macros: These macros provide convenient access to memory-mapped arrays and variables.

### 2. Graph Initialization
  ```C
void init_graph() {
    volatile uint8_t *arr_ptr = (volatile uint8_t *)ARR_BASE;
    // Set all to INVALID first (optional, for safety)
    for (int i = 0; i < 128; i++) arr_ptr[i] = INVALID;
    // Manually initialize the edges
    arr_ptr[0] = 6; arr_ptr[1] = 1; arr_ptr[2] = 255; arr_ptr[3] = 10;
    arr_ptr[4] = 2; arr_ptr[5] = 255; arr_ptr[6] = 0; arr_ptr[7] = 11;
    arr_ptr[8] = 4; arr_ptr[9] = 3; arr_ptr[10] = 5; arr_ptr[11] = 1;
    arr_ptr[12] = 255; arr_ptr[13] = 255; arr_ptr[14] = 2; arr_ptr[15] = 255;
    arr_ptr[16] = 255; arr_ptr[17] = 255; arr_ptr[18] = 255; arr_ptr[19] = 2;
    arr_ptr[20] = 255; arr_ptr[21] = 2; arr_ptr[22] = 255; arr_ptr[23] = 255;
    arr_ptr[24] = 8; arr_ptr[25] = 7; arr_ptr[26] = 9; arr_ptr[27] = 0;
    arr_ptr[28] = 255; arr_ptr[29] = 255; arr_ptr[30] = 6; arr_ptr[31] = 255;
    arr_ptr[32] = 255; arr_ptr[33] = 255; arr_ptr[34] = 255; arr_ptr[35] = 6;
    arr_ptr[36] = 255; arr_ptr[37] = 6; arr_ptr[38] = 255; arr_ptr[39] = 255;
    arr_ptr[40] = 0; arr_ptr[41] = 11; arr_ptr[42] = 26; arr_ptr[43] = 24;
    arr_ptr[44] = 1; arr_ptr[45] = 12; arr_ptr[46] = 10; arr_ptr[47] = 19;
    arr_ptr[48] = 14; arr_ptr[49] = 255; arr_ptr[50] = 11; arr_ptr[51] = 13;
    arr_ptr[52] = 12; arr_ptr[53] = 255; arr_ptr[54] = 255; arr_ptr[55] = 255;
    arr_ptr[56] = 12; arr_ptr[57] = 255; arr_ptr[58] = 15; arr_ptr[59] = 16;
    arr_ptr[60] = 255; arr_ptr[61] = 14; arr_ptr[62] = 255; arr_ptr[63] = 255;
    arr_ptr[64] = 17; arr_ptr[65] = 14; arr_ptr[66] = 18; arr_ptr[67] = 255;
    arr_ptr[68] = 255; arr_ptr[69] = 255; arr_ptr[70] = 255; arr_ptr[71] = 16;
    arr_ptr[72] = 19; arr_ptr[73] = 16; arr_ptr[74] = 21; arr_ptr[75] = 255;
    arr_ptr[76] = 11; arr_ptr[77] = 255; arr_ptr[78] = 20; arr_ptr[79] = 18;
    arr_ptr[80] = 255; arr_ptr[81] = 19; arr_ptr[82] = 255; arr_ptr[83] = 255;
    arr_ptr[84] = 22; arr_ptr[85] = 18; arr_ptr[86] = 23; arr_ptr[87] = 255;
    arr_ptr[88] = 255; arr_ptr[89] = 255; arr_ptr[90] = 255; arr_ptr[91] = 21;
    arr_ptr[92] = 24; arr_ptr[93] = 21; arr_ptr[94] = 30; arr_ptr[95] = 255;
    arr_ptr[96] = 10; arr_ptr[97] = 25; arr_ptr[98] = 255; arr_ptr[99] = 23;
    arr_ptr[100] = 255; arr_ptr[101] = 255; arr_ptr[102] = 24; arr_ptr[103] = 255;
    arr_ptr[104] = 255; arr_ptr[105] = 10; arr_ptr[106] = 28; arr_ptr[107] = 27;
    arr_ptr[108] = 26; arr_ptr[109] = 255; arr_ptr[110] = 255; arr_ptr[111] = 255;
    arr_ptr[112] = 26; arr_ptr[113] = 29; arr_ptr[114] = 255; arr_ptr[115] = 30;
    arr_ptr[116] = 255; arr_ptr[117] = 255; arr_ptr[118] = 28; arr_ptr[119] = 255;
    arr_ptr[120] = 31; arr_ptr[121] = 23; arr_ptr[122] = 28; arr_ptr[123] = 255;
    arr_ptr[124] = 255; arr_ptr[125] = 255; arr_ptr[126] = 255; arr_ptr[127] = 30;
}
```
- Initialization: The graph is manually initialized by writing to the memory-mapped array at ARR_BASE.
- Safety: All entries are first set to INVALID (255) to ensure no accidental valid connections.
- Manual Assignment: Each edge is set explicitly, ensuring the graph structure matches the intended layout.

### 3. Queue Operations
```C
void enqueue(uint8_t node) {
    if (((rear + 1) % NODES) == front) return; // Queue full
    queue[rear] = node;
    rear = (rear + 1) % NODES;
}

uint8_t dequeue() {
    if (front == rear) return INVALID;
    uint8_t node = queue[front];
    front = (front + 1) % NODES;
    return node;
}
```
- Circular Queue: The queue is implemented as a circular buffer to efficiently manage the BFS frontier.
- Enqueue: Adds a node to the end of the queue, checking for overflow.
- Dequeue: Removes and returns the node at the front of the queue, checking for underflow.
- Front and Rear: The **front** pointer points to the next node to be removed (dequeued) from the queue while **rear** points to the next available slot to insert (enqueue) a new node.

### 4. BFS Implementation
#### - Initialization
```c
for (int i = 0; i < NODES; i++) {
    visited[i] = 0;          // Mark all nodes as unvisited
    parent[i] = INVALID;     // Reset parent pointers
}
front = rear = 0;            // Clear queue
```
  - Prepares tracking arrays and queue for a new BFS run.

#### - Start Node Setup

```c
enqueue(START);             // Add start node to queue
visited[START] = 1;         // Mark start node as visited
parent[START] = INVALID;    // Start node has no parent
```
#### - Core Loop: Process Nodes

```c
while (front != rear) {
    uint8_t current = dequeue();  // Get next node to process
```
  - Loop continues until the queue is empty.

#### - Explore Neighbors

```c
for (uint8_t dir = 0; dir < DIRS; dir++) {
    uint8_t next = arr[current][dir];
    if (next != INVALID && next < NODES && !visited[next]) {
        visited[next] = 1;      // Mark neighbor as visited
        parent[next] = current; // Record discovery path
        enqueue(next);          // Add neighbor to queue
    }
}
```
- Checks all 4 directions of the current node.
- Valid neighbors (not INVALID, in range, unvisited) are:
  - Marked as visited
  - Linked to current node via parent[]
  - Added to the queue for future processing

- Termination
  - Loop ends when all reachable nodes are processed.
  - No early exit – continues even after finding target to ensure full traversal.

### 5. Storing Path
```C
void store_path(uint8_t target_node) {
    if (parent[target_node] == INVALID && target_node != START) {
        path_len = 0;
        return;
    }
    uint8_t temp_path[NODES];
    uint8_t len = 0;
    uint8_t current = target_node;
    while (current != INVALID && len < NODES) {
        temp_path[len] = current;
        len++;
        current = parent[current];
    }
    // Check if path exists (should reach start node)
    if (len == 0 || temp_path[len-1] != START) {
        path_len = 0;
        return;
    }
    path_len = len;
    for (uint8_t i = 0; i < len; i++) {
        path[i] = temp_path[len - 1 - i];
    }
}
```
- Check for Valid Path
  - Input: The function takes the target node (the node you want to reach).
  - Validation: It first checks if the target node is unreachable (i.e., its parent is INVALID and it is not the start node). If so, it sets the path length to 0 and exits.

- Backtracking the Path
  - Initialize: A temporary array (temp_path) is used to store the nodes as you backtrack from the target to the start.
  - Loop: Starting from the target node, the function repeatedly looks up the parent of the current node and adds it to the temporary array. This continues until either the start node is reached or the maximum number of nodes is exceeded.

- Check Path Completeness
  - Validation: After backtracking, the function checks if the start node was actually reached (i.e., if the path is valid and complete). If not, it sets the path length to 0 and exits.

- Reverse and Store the Path
  - Since backtracking gives the path in reverse order (from target to start), the function reverses the nodes in the temporary array.
  - The reversed path is then copied into the memory-mapped path array, and its length is saved at the designated memory address

### 6. Other Components:
We can initialize the _start as follows:
```C
void _start() {
    asm volatile("li sp, 0x12000");
    main();
    while(1);
}
```
- In standard C programs, execution typically begins with the main function. However, in bare-metal or embedded systems, the program needs a specific starting point that the hardware or simulator knows to execute first. The `_start` function is traditionally used as the entry point in such environemnts.
- The line asm volatile `("li sp, 0x12000");` sets the stack pointer to a safe memory location, ensuring that function calls, local variables, and interrupts (if any) work correctly.

To use all the functions, we simply use all of them in int main():
```C
int main() {
    init_graph();
    bfs(START);
    store_path(TARGET);
    return 0;
}
```

## Future Developments
We are currently focusing on improving the code to make a robot which can navigate the grid. For that, we will need to do the following:
1. Implement memory-mapped UART communication to communicate start and end points to the processor.
2. Simulate IR sensors which detect if a robot is following the black line in the graph.
3. Modify this code to be more memory friendly by reusing space used by unnecessary arrays and variables.

We will commit any changes done to both this README file and to the C file which contains the code.

## Authors:
- Vaibhav Sharma (@Vaibhav-1023)
- Avni Jharware (@bytebunny2005)
