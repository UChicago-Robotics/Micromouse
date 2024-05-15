/*
#include <Wire.h>
#include "Arduino.h"
#include "const.h"
#include "sensor.h"
#include "motor.h"
#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>*/

#include <stdlib.h>
#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <cmath>
#include <array>
#include <utility>
#include <queue>
#include <vector>
#include <unordered_set>
#include <queue>
#include <stdint.h>
#include <stdexcept>
#include <algorithm>

using namespace std;

typedef pair<int, int> ipair;
typedef vector<pair<int, int>> vpair;

// TODO: confirm this shit runs on arduino (uses a lot of c++17 features)

#define WALL 1
#define EMPTY 0
#define UNKNOWN 2
#define UNVISITED 2
#define VISITED 3

#define RIGHT 0
#define DOWN 1
#define LEFT 2
#define UP 3
#define MOVE 4 

#define MAZE_SIZE 8

int index(int ri, int ci) {
    return ri * MAZE_SIZE + ci;
}

vector<int> get_2d_pos(int index) {
    return {(int) (index / MAZE_SIZE), index % MAZE_SIZE};
}

// used to sort paths based on L1 norm of points
int d2c(int index) {
    int r = get_2d_pos(index)[0], c = get_2d_pos(index)[1];
    int h = MAZE_SIZE/2;
    return min(abs(r-h-1),abs(r-h)) + min(abs(c-h-1),abs(c-h));
}

bool in_bounds(int r, int c) {
    return (0 <= r && 0 <= c && r < MAZE_SIZE && c < MAZE_SIZE);
}

class Navigator{
private:
    void print_maze(const uint8_t* horizontalWalls, const uint8_t* verticalWalls, const vector<char>& characters = {}) {
    int rows = MAZE_SIZE, cols = MAZE_SIZE;

    cout << '+';
    for (int col = 0; col < cols; ++col) {
        cout << "---+";
    }
    cout << endl;

    // Print the maze rows with walls
    for (int row = 0; row < rows; ++row) {
        // Print vertical walls and cells
        cout << '|';
        for (int col = 0; col < cols; ++col) {
            // Print character on the square if specified
            if (!characters.empty()) {
                cout << ' ' << characters[index(row, col)] << ' ';
            } else {
                cout << "   "; // Print an empty cell
            }

            // Print vertical wall if present
            if ((col < cols - 1 && (verticalWalls[row * cols + col] & 0x01)) || col == cols-1) {
                cout << '|';
            } else {
                cout << ' ';
            }
        }
        cout << endl;

        // Print horizontal walls and bottom border
        cout << '+';
        for (int col = 0; col < cols; ++col) {
            // Print horizontal wall if present
            if ((horizontalWalls[index(row, col)] & 0x01) || row == rows-1) {
                cout << "---";
            } else {
                cout << "   ";
            }
            cout << '+';
        }
        cout << endl;
    }
    cout << endl;
}

public:
    int mouse_r = 0;
    int mouse_c = 0;
    int dir = RIGHT;
        
    uint8_t* memV_maze;
    uint8_t* memH_maze;
    uint8_t* mem_maze;

    uint8_t* realV_maze;
    uint8_t* realH_maze;
    uint8_t* real_maze;

    Navigator() {
        
        memV_maze = (uint8_t*) calloc(sizeof(char), MAZE_SIZE*MAZE_SIZE-1);
        memH_maze = (uint8_t*) calloc(sizeof(char), MAZE_SIZE*MAZE_SIZE-1);
        mem_maze = (uint8_t*) calloc(sizeof(char), MAZE_SIZE*MAZE_SIZE);

        realV_maze = (uint8_t*) calloc(sizeof(char), MAZE_SIZE*MAZE_SIZE-1);
        realH_maze = (uint8_t*) calloc(sizeof(char), MAZE_SIZE*MAZE_SIZE-1);
        real_maze = (uint8_t*) calloc(sizeof(char), MAZE_SIZE*MAZE_SIZE);

        uint8_t HW[7][8] = { // horizontal walls
            {1,1,1,1,0,1,0,1},
            {1,0,1,1,0,0,1,1},
            {0,0,1,1,1,1,0,0},
            {0,0,0,0,0,0,1,0},
            {0,0,0,1,1,0,0,0},
            {0,0,0,1,1,1,0,0},
            {0,0,1,0,1,0,0,0}
        };
        uint8_t VW[8][7] = { // vertical walls
            {0,0,0,0,0,0,1},
            {0,0,0,1,1,1,0},
            {1,0,0,0,0,0,1},
            {0,1,1,0,0,1,0},
            {1,1,1,0,1,1,1},
            {0,1,0,0,0,0,0},
            {1,1,1,0,0,1,1},
            {1,0,0,1,1,0,1}
        };

        for (int i = 0; i < MAZE_SIZE; i++) {
            for (int j = 0; j < MAZE_SIZE; j++) {
                mem_maze[index(i, j)] = UNVISITED;
                real_maze[index(i,j)] = 0;
            }
        }
        for (int i = 0; i < MAZE_SIZE-1;i++) {
            for (int j = 0; j < MAZE_SIZE; j++) {
                realV_maze[index(j,i)] = VW[j][i];
                realH_maze[index(i,j)] = HW[i][j];
                memV_maze[index(j,i)] = UNVISITED;
                memH_maze[index(i,j)] = UNVISITED;
            }
        }

        auto walls = sense();
        sense_update(walls);
    }

    void print_real_maze() {
        print_maze(realH_maze,realV_maze);
    }

    inline void print_mem_maze() {
        vector<char> visited_arr(MAZE_SIZE*MAZE_SIZE);

        vector<int> adj = get_adj(mouse_r, mouse_c);

        for (int i = 0; i < MAZE_SIZE*MAZE_SIZE; i++) {
            if (mem_maze[i] == VISITED) visited_arr[i] = ' ';
            if (mem_maze[i] == UNVISITED) visited_arr[i] = '.';
            if (index(mouse_r, mouse_c) == i) {
                if (dir == UP) visited_arr[i] = '^';
                else if (dir == RIGHT) visited_arr[i] = '>';
                else if (dir == LEFT) visited_arr[i] = '<';
                else if (dir == DOWN) visited_arr[i] = 'v';
            }
            
            // optional: print ADJ
            if (find(adj.begin(), adj.end(), i) != adj.end()) visited_arr[i] = 'A';
        }

        print_maze(memH_maze,memV_maze,visited_arr);
    }
    
    bool done() {
        int h = MAZE_SIZE/2;
        return (mouse_r == h || mouse_r == h-1) && (mouse_c == h || mouse_c == h-1);
    }

    /**
     * @brief Get the adj nodes from current position. Note that we only consider
     * known walls TODO: Update to read from memory not is_wall_present
     * 
     * @return vector<int> 
     */
    vector<int> get_adj(int r, int c) {
        vector<int> adj_nodes;

        if (!is_wall_present(memV_maze, r, c) && in_bounds(r, c+1)) {
            adj_nodes.push_back(index(r, c + 1));
        } 
        if (!is_wall_present(memV_maze, r, c - 1) && in_bounds(r, c-1)) {
            adj_nodes.push_back(index(r, c - 1));
        } 
        if (!is_wall_present(memH_maze, r, c) && in_bounds(r+1, c)) {
            adj_nodes.push_back(index(r + 1, c));
        } 
        if (!is_wall_present(memH_maze, r - 1, c) && in_bounds(r-1, c)) {
            adj_nodes.push_back(index(r - 1, c));
        } 

        return adj_nodes;
    }

    vector<int> get_adj(int index) {
        return get_adj((int) (index / MAZE_SIZE), index % MAZE_SIZE);
    }

    // Function to check if a wall is present at a specific position in the maze
    // this is the only function that needs sensor update
    bool is_wall_present(const uint8_t* maze, int row, int col) {
        // Check for boundary walls
        if (row < 0 || row >= MAZE_SIZE || col < 0 || col >= MAZE_SIZE) {
            return true; // Treat out-of-bound positions as walls
        }

        return maze[index(row, col)] != EMPTY;
    }

    /**
     * @brief emulator for the mouse sensing the physical walls in the current direction
     * RELATIVE TO THE MOUSE.
     * 
     * @return std::array<bool,3> : a boolean representing the walls to the 
     * current left, center, right
     */
    array<bool,3> sense() {
        array<bool, 3> walls; // Initialize with no walls (left, center, right)
        int r = mouse_r, c = mouse_c;

        switch (dir) {
            case UP: 
                walls[0] = is_wall_present(realV_maze, r, c - 1);
                walls[1] = is_wall_present(realH_maze, r - 1, c);
                walls[2] = is_wall_present(realV_maze, r, c);
                break;
            case DOWN:
                walls[0] = is_wall_present(realV_maze, r, c);
                walls[1] = is_wall_present(realH_maze, r, c);
                walls[2] = is_wall_present(realV_maze, r, c - 1);
                break;
            case LEFT:
                walls[0] = is_wall_present(realH_maze, r, c);
                walls[1] = is_wall_present(realV_maze, r, c - 1);
                walls[2] = is_wall_present(realH_maze, r - 1, c);
                break;
            case RIGHT:
                walls[0] = is_wall_present(realH_maze, r - 1, c);
                walls[1] = is_wall_present(realV_maze, r, c);
                walls[2] = is_wall_present(realH_maze, r, c);
                break;
        }

        return walls;
    }

    /**
     * @brief Finds the path to the nearest unvisited node that is closest to the center
     * 
     * @param r1 
     * @param c1 
     * @param r2 
     * @param c2 
     * @return std::vector<int> 
     */
    vector<int> floodfill(int src) {
        int n = MAZE_SIZE*MAZE_SIZE;

        // we use the 1d index of each node because c++ pairs aren't hashable
        unordered_set<int> visited = {src};
        vector<int> prev(n, -1);
        vector<int> dist(n, -1);

        vector<int> cur_layer = {src};
        vector<int> next_layer;

        int closest = -1;

        while (!cur_layer.empty()) {
            for (int cur: cur_layer) {
                for (int adj : get_adj(cur)) {

                    // only check adj nodes if they have not been reached in our
                    // floodfill search and the node we're searching from is visted
                    if (!visited.count(adj) && mem_maze[cur] == VISITED) {
                        prev[adj] = cur;

                        next_layer.push_back(adj);
                        visited.insert(adj);

                        // write as closest unvisited
                        if (closest == -1 && mem_maze[adj] == UNVISITED)
                            closest = adj;
                    }
                }
            }

            cur_layer = next_layer;
            next_layer = {};
        }

        if (closest == -1) return {};

        // find nearested unvisited node with shortest distance to center
        /*
        vector<int> closest_to_center = {0, INT16_MAX};
        for (int v : visited) {
            if (mem_maze[v] == UNVISITED && d2c(v) < closest_to_center[1])
                closest_to_center = {v, d2c(v)};
        }
        */

        // find path from src to dst
        vector<int> path;
        
        int dst = closest;
        int cur = dst;

        while (cur != src) {
            path.push_back(cur);
            cur = prev[cur];
        }

        reverse(path.begin(), path.end());
        return path;
    }

    /**
     * @brief Generates a list of instructions from a path
     * 
     */
    vector<int> generate_instr(vector<int> path) {
        int cdir = dir;
        int cr = mouse_r, cc = mouse_c;

        vector<int> instr;

        for (int p : path) {
            int nr = get_2d_pos(p)[0];
            int nc = get_2d_pos(p)[1];

            if (nr == cr - 1 && nc == cc) {        // Face up
                if (cdir == DOWN) instr.insert(instr.end(), {RIGHT, RIGHT});
                if (cdir == LEFT) instr.push_back(RIGHT);
                if (cdir == RIGHT) instr.push_back(LEFT);
                cdir = UP;
            } else if (nr == cr + 1 && nc == cc) { // Face down
                if (cdir == UP) instr.insert(instr.end(), {RIGHT, RIGHT});
                if (cdir == LEFT) instr.push_back(LEFT);
                if (cdir == RIGHT) instr.push_back(RIGHT);
                cdir = DOWN;
            } else if (nr == cr && nc == cc + 1) { // Face right
                if (cdir == LEFT) instr.insert(instr.end(), {RIGHT, RIGHT});
                if (cdir == UP) instr.push_back(RIGHT);
                if (cdir == DOWN) instr.push_back(LEFT);
                cdir = RIGHT;
            } else if (nr == cr && nc == cc - 1) { // Face left
                if (cdir == RIGHT) instr.insert(instr.end(), {RIGHT, RIGHT});
                if (cdir == UP) instr.push_back(LEFT);
                if (cdir == DOWN) instr.push_back(RIGHT);
                cdir = LEFT;
            }

            instr.push_back(MOVE);  // Move forward

            // Update current row and column
            cr = nr;
            cc = nc;
        }

        return instr;
    }

    /**
     * @brief updates memory array based on observation at a point. Memory is always
     * up to date. I have no idea how or why this works but it seems to hold up after
     * testing so I'm just not gonna fuck with it.
     * 
     * @param w: walls
     */
    void sense_update(std::array<bool,3> w) {
        mem_maze[index(mouse_r,mouse_c)] = VISITED;

        std::array<int,3> rs = {0,0,0};
        std::array<int,3> cs = {0,0,0};
        if (dir == 0) { // returns h,v,h
            rs = {-1,0,0};
            cs = {0,0,0};
        } else if (dir == 1) { // returns v,h,v
            rs = {0,0,0};
            cs = {0,0,-1};
        } else if (dir == 2) { // returns h,v,h
            rs = {0,0,-1};
            cs = {0,-1,0};
        } else { // returns v,h,v
            rs = {0,-1,0};
            cs = {-1,0,0};
        }
        for (int i = 0; i < 3; i++) {
            int ri = rs[i] + mouse_r;
            int ci = cs[i] + mouse_c;
            if (ri < 0 || ci < 0) {
                continue;
            }
            if ((dir % 2 == 0 && i % 2 ==1) || (dir % 2 == 1 && i % 2 == 0)) { // left and right so in bounds (N-1,N) (N,N-1) (N-1,N)
                if (ri >= MAZE_SIZE || ci >= MAZE_SIZE-1) {
                    continue;
                }
                memV_maze[index(ri,ci)] = w[i];
            } else {
                if (ri >= MAZE_SIZE-1 || ci >= MAZE_SIZE ) {
                    continue;
                }
                memH_maze[index(ri,ci)] = w[i];
            }
        }
    }

    // Movement API

    void turn_left() {
        if (this->dir == RIGHT) this->dir = UP;
        else if (this->dir == UP) this->dir = LEFT;
        else if (this->dir == LEFT) this->dir = DOWN;
        else if (this->dir == DOWN) this->dir = RIGHT;
    }

    void turn_right() {
        if (this->dir == RIGHT) this->dir = DOWN;
        else if (this->dir == DOWN) this->dir = LEFT;
        else if (this->dir == LEFT) this->dir = UP;
        else if (this->dir == UP) this->dir = RIGHT;
    }

    void move_straight() {
        if (dir == RIGHT) mouse_c++;
        if (dir == LEFT) mouse_c--;
        if (dir == UP) mouse_r--;
        if (dir == DOWN) mouse_r++;
    } 

};

void adj_test() {
    Navigator n;

    n.print_real_maze();

    auto walls = n.sense();
    n.sense_update(walls);

    n.mouse_r = 3;
    n.mouse_c = 3;
    n.dir = UP;
    walls = n.sense();
    n.sense_update(walls);
    n.print_mem_maze();
}

void print_instr(vector<int> instr) {
    for (int i : instr) {
        if (i == RIGHT) printf("RIGHT ");
        if (i == LEFT) printf("LEFT ");
        if (i == MOVE) printf("MOVE ");
    }
    printf("\n");
}

void print_path(vector<int> path) {
    for (int p : path) {
        printf("(%d, %d) ", get_2d_pos(p)[0], get_2d_pos(p)[1]);
    }
    printf("\n");
}

void floodfill_test() {
    Navigator n;
    n.print_real_maze();

    for (int i = 0; i < 64; i++) {
        vector<int> path = n.floodfill(index(n.mouse_r, n.mouse_c));
        vector<int> instr = n.generate_instr(path);
        print_instr(instr);
        print_path(path);

        for (int in : instr) {
            if (in == LEFT) n.turn_left();
            if (in == RIGHT) n.turn_right();
            if (in == MOVE) n.move_straight();
            
            auto walls = n.sense();
            n.sense_update(walls);
        }

        n.print_mem_maze();
    }

    n.print_real_maze();
}

int main() {
    floodfill_test();
}