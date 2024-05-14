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

#define MAZE_SIZE 8

int index(int ri, int ci) {
    return ri * MAZE_SIZE + ci;
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

    int d2c(int r, int c) {
        // used to sort paths based on L1 norm of points
        int h = MAZE_SIZE/2;
        return min(abs(r-h-1),abs(r-h)) + min(abs(c-h-1),abs(c-h));
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
            if (index(mouse_r, mouse_c) == i) visited_arr[i] = 'M';
            
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
     * known walls
     * 
     * @return vector<int> 
     */
    vector<int> get_adj(int r, int c) {
        vector<int> adj_nodes;

        if (!is_wall_present(memV_maze, r, c) ) {
            adj_nodes.push_back(index(r, c + 1));
        } 
        if (!is_wall_present(memV_maze, r, c - 1) ) {
            adj_nodes.push_back(index(r, c - 1));
        } 
        if (!is_wall_present(memH_maze, r, c) ) {
            adj_nodes.push_back(index(r + 1, c));
        } 
        if (!is_wall_present(memH_maze, r - 1, c) ) {
            adj_nodes.push_back(index(r - 1, c));
        } 

        return adj_nodes;
    }

    // Function to check if a wall is present at a specific position in the maze
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
                walls[0] = is_wall_present(realV_maze, r, c - 1); // Left wall
                walls[1] = is_wall_present(realH_maze, r - 1, c);     // Center wall
                walls[2] = is_wall_present(realV_maze, r, c);     // Right wall
                break;
            case DOWN:
                walls[0] = is_wall_present(realV_maze, r, c);     // Left wall
                walls[1] = is_wall_present(realH_maze, r, c); // Center wall
                walls[2] = is_wall_present(realV_maze, r, c - 1);     // Right wall
                break;
            case LEFT:
                walls[0] = is_wall_present(realH_maze, r, c);       // Left wall
                walls[1] = is_wall_present(realV_maze, r, c - 1);   // Center wall
                walls[2] = is_wall_present(realH_maze, r - 1, c); // Right wall
                break;
            case RIGHT:
                walls[0] = is_wall_present(realH_maze, r - 1, c);     // Left wall
                walls[1] = is_wall_present(realV_maze, r, c);     // Center wall
                walls[2] = is_wall_present(realH_maze, r, c); // Right wall
                break;
        }

        return walls;
}

    /**
     * @brief updates memory array based on observation at a point. Memory is always
     * up to date.
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


};


void adj_test() {
    Navigator n;

    n.print_real_maze();
    n.print_mem_maze();

    auto walls = n.sense();
    n.sense_update(walls);

    n.mouse_c = 0;
    n.mouse_r = 0;
    walls = n.sense();
    n.sense_update(walls);
    n.print_mem_maze();
}

int main() {
    adj_test();
}