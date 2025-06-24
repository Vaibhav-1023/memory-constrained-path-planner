#include <stdint.h>

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

void bfs(uint8_t start_node) {
    for (int i = 0; i < NODES; i++) {
        visited[i] = 0;
        parent[i] = INVALID;
    }
    front = rear = 0;

    enqueue(start_node);
    visited[start_node] = 1;
    parent[start_node] = INVALID;

    while (front != rear) {
        uint8_t current = dequeue();
        for (uint8_t dir = 0; dir < DIRS; dir++) {
            uint8_t next = arr[current][dir];
            if (next != INVALID && next < NODES && !visited[next]) {
                visited[next] = 1;
                parent[next] = current;
                enqueue(next);
            }
        }
    }
}

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

void _start() {
    asm volatile("li sp, 0x12000");
    main();
    while(1);
}

int main() {
    init_graph();
    bfs(START);
    store_path(TARGET);
    return 0;
}
