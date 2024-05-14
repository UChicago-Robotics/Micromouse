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
using namespace std;

typedef pair<int, int> ipair;
typedef vector<pair<int, int>> vpair;

// TODO: make into class
// TODO: confirm this shit runs on arduino (uses a lot of c++17 features)

//#define empty_f 0
#define WALL 1
#define EMPTY 0
#define UNVISITED 2
#define VISITED 3
#define MAZE_SIZE 8



int index(int ri, int ci) {
    return ri * MAZE_SIZE + ci;
}

/**
 * @brief returns the 2d-position from the 1d-index
 */
pair<int, int> get2Dpos(int index) {
    return {(int) (index / MAZE_SIZE), index % MAZE_SIZE};
}
/**
 * @brief get the 1d-index from 2d pos
 * 
 * @param coord 
 * @return int 
 */
int get1Dpos(ipair c) {
    return index(c.first, c.second);
}


class Navigator{
private:
    void print_maze(uint8_t * arrH, uint8_t * arrV, uint8_t * arrVis) {
    // start line
    std::cout << " ";
    for (int i = 0; i < MAZE_SIZE; i++) {
        std::cout << "__";
    }
    std::cout << " " << std::endl;

    for (int i = 0; i < MAZE_SIZE; i++) { // row
        //vert walls
        std::cout << "|";
        for (int j = 0; j < MAZE_SIZE; j++) {
            switch((int)arrVis[index(i,j)]) {
                case 0:
                    std::cout << "o";
                    break;
                case 1:
                    std::cout << "X";
                    break;
                case 2:
                    std::cout << ".";
            }
            if (j == MAZE_SIZE-1) {
                break;
            }
            switch((int)arrV[index(i, j)]) {
                case 0:
                    std::cout << " ";
                    break;
                case 1:
                    std::cout << "|";
                    break;
                case 2:
                    std::cout << "?";
                    break;
            }
        }
        std::cout << "|" << std::endl;
        // horizontal wall
        if (i == MAZE_SIZE-1) {
            break;
        }
        std::cout << "|";
        for (int j = 0; j < MAZE_SIZE; j++) {
            switch((int)arrH[index(i, j)]) {
                case 0:
                    if (j == MAZE_SIZE -1) {
                        std::cout << " |";
                    } else {
                        std::cout << "  ";\
                    }
                    break;
                case 1:
                    if (j == MAZE_SIZE -1) {
                        std::cout << "-|";
                    } else {
                        std::cout << "--";\
                    }
                    break;
                case 2:
                    std::cout << "??";
                    break;
            }
        }
        std::cout << std::endl;
    }
    // end line
    std::cout << " ";
    for (int i = 0; i < MAZE_SIZE; i++) {
        std::cout << "--";
    }
    std::cout << " " << std::endl;
}


public:
    int r = 0;
    int c = 0;
    int dir = 0; // 0 = right, 1 = down, 2 = left, 3 = up
        
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
    }

    int d2c(int r, int c) {
        // used to sort paths based on L1 norm of points
        int h = MAZE_SIZE/2;
        return min(abs(r-h-1),abs(r-h)) + min(abs(c-h-1),abs(c-h));
    }

    void print_real_maze() {
        print_maze(realH_maze,realV_maze,real_maze);
    }

    void print_mem_maze() {
        print_maze(memH_maze,memV_maze,mem_maze);
    }
    
    bool done() {
        int h = MAZE_SIZE/2;
        return (r == h || r == h-1) && (c == h || c == h-1);
    }

    /**
     * @brief Get adjacent visitable nodes from position.
     * 
     * @param pos 
     * @param only_visited only return nodes that have been visited
     * @return vector<ipair> 
     */
    vector<ipair> get_adj(ipair pos, bool only_visited = false) {
        int r = pos.first, c = pos.second;
        vector<ipair> adj;

        // check adjacent nodes that don't have blocking walls
        if (memH_maze[index(r-1, c)] != WALL) adj.push_back({r-1, c });
        if (memH_maze[index(r, c)] != WALL) adj.push_back({r+1, c});
        
        if (memV_maze[index(r, c-1)] != WALL) adj.push_back({r, c-1});
        if (memV_maze[index(r, c)] != WALL) adj.push_back({r, c+1});
        
        int visited_bitmask = !(only_visited << 31);
        vector<ipair> res;

        // filter out adj nodes with invalid coords
        for (auto a : adj) {
            if (
                0 <= a.first 
                && 0 <= a.second 
                && a.first < MAZE_SIZE 
                && a.second < MAZE_SIZE
                && ((mem_maze[index(a.first, a.second)] == VISITED) | visited_bitmask) // possible bug
            ) {
                res.push_back(a);
            }
        }
        
        return res;
    }

    /**
     * @brief Helper function to "unpack" the result of dijkstra's into a list of
     * indices. 
     * 
     * @param parent 
     * @param dest 
     * @return std::vector<int> path as list of cell indecies
     */
    std::vector<int> getPath(const std::vector<int>& parent, int dest) {

    /**
     * @brief Finds the shortest path between two nodes, returns the result as a list
     * of 1d-indices. Note that it can only travel on visited nodes. 
     * 
     * @param r1 
     * @param c1 
     * @param r2 
     * @param c2 
     * @return std::vector<int> 
     */
    vector<int> shortest_path(int r1, int c1, int r2, int c2) {
        int n = MAZE_SIZE*MAZE_SIZE;

        // we use the 1d index of each node because c++ pairs aren't hashable lol
        unordered_set<int> visited;
        vector<int> prev(n, -1);
        vector<int> dist(n, -1);

        vector<int> cur_layer = {index(r1, c1)};
        visited.insert(index(r1, c1));
        dist[index(r1, c1)] = 0;
        int cdist = 0;

        while (!cur_layer.empty()) {
            vector<int> next_layer;
            cdist++;

            for (int cur: cur_layer) {
                for (ipair adj : get_adj(get2Dpos(cur))) {
                    int adj_index = get1Dpos(adj);

                    if (visited.count(adj_index) == 0) {
                        next_layer.push_back(adj_index);
                        prev[adj_index] = cur;
                        dist[adj_index] = cdist;
                        visited.insert(adj_index);
                    }
                }
            }

            cur_layer = next_layer;
        }

        return getPath(prev, index(r2, c2));
    }
};

/**
 * @brief converts list of indices (path ie 1,2,3,4,12) to instructions to get from A to B 
 * ex: (0,0,0,1) --> (straight straight straight right+straight)
 * 
 * @param path 1-d array representing a path from node a --> b
 * @return std::vector<int> 
 */
std::vector<int> path2nav(std::vector<int> path) {
    std::vector<int> nav = {};
    for (int i = 0; i < path.size()-1; i++) {
        int move = -100;
        switch(path[i+1]-path[i]) {
            case 1:
                nav.push_back(0);
                break;
            case MAZE_SIZE:
                nav.push_back(1);
                break;
            case -MAZE_SIZE:
                nav.push_back(-1);
                break;
            default:
                nav.push_back(-100);
                std::cout << "YOU FUCKED UP" << std::endl;   
        }
    }
    return nav;
}

/**
 * @brief Floodfill algorithm, finds the nearest unvisited node. If there are 2 nodes
 * that are equidistant, we choose the node that's closer to the center.
 * 
 * @param r
 * @param c
 * @return std::vector<int> 
 */
std::vector<int> floodfill(int r, int c) {
    int target_r; 
    int target_c;

    vector<pair<int, int>> unvisited;
    vector<pair<int, int>> cur_layer = {{r, c}};

    // BFS to find nearest unvisited node
    while (unvisited.empty()) {
        vpair next_layer;

        for (ipair n : cur_layer) {
            for (auto a : get_adj(n)) {
                    if (mem_maze[index(a.first, a.second)] == VISITED) {
                        next_layer.push_back(a);
                    
                    // if node is unvisited, add to queue
                    } else if (mem_maze[index(a.first, a.second)] == UNVISITED) {
                        unvisited.push_back(a);
                    }
                }
            }
         cur_layer = next_layer;
    }

    // TODO: select based on better scheme
    target_r = unvisited[0].first, target_c = unvisited[0].second;
    //printf("(%d, %d) --> (%d, %d)\n", r, c, target_r, target_c);
    return shortest_path(r, c, target_r, target_c);
}

/**
 * @brief emulator for the mouse sensing the physical walls in the current direction
 * RELATIVE TO THE MOUSE.
 * 
 * @return std::array<bool,3> : a boolean representing the walls to the 
 * current left, center, right
 */
std::array<bool,3> sense(int r, int c, int dir) {
    // I'm a lazy piece of shit, so I'm hardcoding this and it's nasty af --> but it works
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
    std::array<bool,3> walls = {false,false,false};
    for (int i = 0; i < 3; i++) {
        int ri = rs[i] + r;
        int ci = cs[i] + c;
        if (ri < 0 || ci < 0) {
          walls[i] = true; // outside wall
          continue;
        }
        if ((dir % 2 == 0 && i % 2 ==1) || (dir % 2 == 1 && i % 2 == 0)) { // left and right so in bounds (N-1,N) (N,N-1) (N-1,N)
          // verticals
          if (ri >= MAZE_SIZE || ci >= MAZE_SIZE -1) {
            walls[i] = true; // outside wall
            continue;
          }
          walls[i] = realV_maze[index(ri,ci)];
        }
        else {
          // horizontals
          if (ri >= MAZE_SIZE-1 || ci >= MAZE_SIZE ) {
            walls[i] = true; // outside wall
            continue;
          }
          walls[i] = realH_maze[index(ri,ci)];
        }
    }
    return walls;

}

/**
 * @brief updates memory array based on observation at a point. Memory is always
 * up to date.
 * 
 * @param w 
 */
void sense_update(std::array<bool,3> w, int r, int c, int dir) {
    mem_maze[index(r,c)] = 0;
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
        int ri = rs[i] + r;
        int ci = cs[i] + c;
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

int main() {

    /*
    print_real_maze();
    print_mem_maze();
    //<\example maze>

    // <examples of subroutines drivepath, sense, done, etc>
    // driving navigation demos
    std::vector<int> nav = {0,0,0,1,0,1,-1};
    drivenav(nav); // (0,0) -> (3,2)
    std::cout << r << " " << c << std::endl;
    nav = {2,0};
    drivenav(nav); // (3,2) -> (1,2)
    std::cout << r << " " << c << std::endl;

    // sensing walls
    r = 4;
    c = 5;
    dir = 0;
    std::array<bool,3> s = sense();
    std::cout << s[0] << " " << s[1] << " " << s[2] << std::endl;

    // done ie if in final cells
    r = 3;
    c = 5;
    std::cout << done() << std::endl;

    // sense update
    r = 0;
    c = 0;
    dir = 0;
    sense_update(sense());
    print_mem_maze();
    c = 1;
    sense_update(sense());
    print_mem_maze();

    // connection
    std::cout << connected(0,1) << std::endl;

    // dijkstra
    c = 2;
    sense_update(sense());
    c = 3;
    sense_update(sense());
    c = 4;
    sense_update(sense());
    c = 5;
    sense_update(sense());
    c = 6;
    sense_update(sense());
    r = 1;
    dir = 1;
    sense_update(sense());
    c = 4;
    sense_update(sense());
    print_mem_maze();
    std::vector<int> p = dijkstra(0,0,1,4);
    for (int i = 0; i < p.size(); i ++) {
        std::cout << p[i] << " ";
    }
    std::cout << std::endl;
    
    // convert path to navigation instructions
    std::vector<int> n = path2nav(p);
        for (int i = 0; i < n.size(); i ++) {
        std::cout << n[i] << " ";
    }
    std::cout << std::endl;
    // <\examples of subroutines navigate, sense, done, dijkstra>
    */

    for (int i = 0; i < 100; i++) {
        auto walls = sense();
        sense_update(walls);

        int prev_r = r;
        int prev_c = c;

        vector<int> path = floodfill(r, c);
        ipair nn = get2Dpos(path[path.size()-1]);

        if (prev_r > nn.first) {
            dir = 1; // down
        } if (prev_r < nn.first) {
            dir = 3; // up
        } if (prev_c < nn.second) {
            dir = 0; // right
        } if (prev_c > nn.second) {
            dir = 2; // left
        }

        // move to new position
        r = nn.first;
        c = nn.second;

        printf("%d, %d, %d \n", r, c, dir);
        print_maze(memH_maze, memV_maze, mem_maze);
        printf("\n");
    }
}