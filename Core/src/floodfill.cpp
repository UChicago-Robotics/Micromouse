/*
#include <Wire.h>
#include "Arduino.h"
#include "const.h"
#include "sensor.h"
#include "motor.h"
#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>*/

#include <stdlib.h>     /* calloc, exit, free */
#include <iostream>
#include <cmath>

#include <utility>
#include <queue>
#include <vector>
#include <unordered_set>

#include <stdexcept>

// TODO: dijkstra's to get shortest path to next place in floodfill

//#define empty_f 0
#define WALL 1
#define UNVISITED 0
#define VISITED 2

struct pair_hash {
    inline std::size_t operator()(const std::pair<int,int> & v) const {
        return v.first*31+v.second;
    }
};

class FloodFill {
private:
    void print_maze(uint8_t * arr) {
        for (int i = 0; i < maze_size; i++) {
            for (int j = 0; j < maze_size; j++) {
                std::cout << (int)arr[index(i, j)] << " ";
            }
            std::cout << std::endl;
        }
    }

public:
    int maze_size;
    uint8_t* mem_maze;
    uint8_t* real_maze;

    FloodFill() {
        maze_size = 16;

        mem_maze = (uint8_t*) calloc(sizeof(char), maze_size*maze_size);
        real_maze = (uint8_t*) calloc(sizeof(char), maze_size*maze_size);
        
        uint8_t intput_arr[16][16] = {
            {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
            {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
            {1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1},
            {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
            {1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,1},
            {1,1,1,0,1,1,1,1,1,0,1,1,1,0,1,1},
            {1,1,1,0,1,1,1,1,1,0,1,1,1,0,1,1},
            {1,1,1,1,1,1,1,1,1,0,1,1,1,0,1,1},
            {1,1,1,0,0,0,0,0,0,0,0,0,0,0,1,1},
            {1,1,1,0,1,1,1,1,1,0,1,1,1,1,1,1},
            {1,1,1,0,1,1,1,1,1,0,1,1,1,1,1,1},
            {1,1,1,0,1,1,1,1,1,0,1,1,1,1,1,1},
            {1,1,1,0,0,0,0,0,0,0,1,1,1,1,1,1},
            {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
            {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
            {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
        };

        for (int i = 0; i < maze_size; i++) {
            for(int j = 0; j < maze_size; j++) {
                real_maze[index(i,j)] = intput_arr[i][j];
                mem_maze[index(i, j)] = UNVISITED;
            }
        }
    }

    // return adjacent nodes that can be visited
    std::vector<std::pair<int, int>> get_adj(int r, int c) {
        if (r < 0 || c < 0  || r > maze_size - 1 || c > maze_size - 1)
            throw std::invalid_argument("Maze boundaries exceeded");
        else if (real_maze[index(r, c)] == 1)
            throw std::invalid_argument("Mouse is in a wall lmao");
        
        std::vector<std::pair<int, int>> v;
        std::vector<std::pair<int, int>> res;

        v.push_back({r+1, c});
        v.push_back({r-1, c});
        v.push_back({r, c+1});
        v.push_back({r, c-1});

        for (auto n : v) {
            if (
                0 <= n.first 
                && 0 <= n.second 
                && n.first < maze_size 
                && n.second < maze_size
            ) {
                // mimic sensor input: check if adjacent squares are walls,
                // write values to memory buffer
                if (real_maze[index(n.first, n.second)] == 0) {
                    res.push_back(n);
                } else {
                    mem_maze[index(n.first, n.second)] = WALL;
                }
            }
        }

        return res;
    }
    
    void floodfill(int target_r, int target_c) {
        int c_row = 1, c_col = 1;

        std::vector<std::pair<int, int>> visit_stack;
        visit_stack.push_back({c_row, c_col});

        while (!visit_stack.empty()) {
            // backtrack to most recentley visited node
            auto cur = visit_stack[visit_stack.size()-1];
            if (mem_maze[index(cur.first, cur.second)] == VISITED) {
                visit_stack.pop_back();
                continue;
            }

            c_row = cur.first, c_col = cur.second;
            visit_stack.pop_back();

            std::cout << c_row << ", " << c_col << "\n";
            mem_maze[index(c_row, c_col)] = VISITED;
            print_mem_maze();
            
            // push unvisited adj nodes to stack
            for (auto adj : get_adj(cur.first, cur.second)) {
                if (mem_maze[index(adj.first, adj.second)] != VISITED) {
                    visit_stack.push_back(adj);
                }
            }
        }
    }

    int index(int r, int c) const {
        return r * maze_size + c;
    }

    void print_real_maze() {
        print_maze(real_maze);
    }

    void print_mem_maze() {
        print_maze(mem_maze);
    }
};

int main() {
    FloodFill a;
    //a.print_real_maze();
    a.floodfill(1, 1);
}