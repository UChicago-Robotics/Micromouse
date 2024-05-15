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

// TODO: confirm this shit runs on arduino (uses a lot of c++17 features)

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
    int mouse_r = 0;
    int mouse_c = 0;
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

        auto walls = sense();
        sense_update(walls);
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
        return (mouse_r == h || mouse_r == h-1) && (mouse_c == h || mouse_c == h-1);
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
        
        auto walls = sense();
        sense_update(walls);

        // check adjacent nodes that don't have blocking walls
        if (memH_maze[index(r-1, c)] == EMPTY) adj.push_back({r-1, c });

        if (memH_maze[index(r, c)] == EMPTY) adj.push_back({r+1, c});
    
        if (memV_maze[index(r, c+1)] == EMPTY) adj.push_back({r, c-1});
    
        if (memV_maze[index(r, c)] == EMPTY) adj.push_back({r, c+1});
        
        vector<ipair> res;

        // filter out adj nodes with invalid coords
        for (auto a : adj) {
            if (
                0 <= a.first 
                && 0 <= a.second 
                && a.first < MAZE_SIZE
                && a.second < MAZE_SIZE
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
        // i've heard microcontrollers hate recursive stack pressure, but i'm too lazy to fix
        std::vector<int> path;
        if (parent[dest] == -1)
            return path;
        path = getPath(parent, parent[dest]);
        path.push_back(dest);
        return path;
    }

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
    vector<ipair> shortest_path(int r1, int c1, int r2, int c2) {
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

        vector<int> index_path = getPath(prev, index(r2, c2));
        vector<ipair> res;

        for (auto i: index_path) res.push_back(get2Dpos(i));
        return res;
    }

    /**
     * @brief emulator for the mouse sensing the physical walls in the current direction
     * RELATIVE TO THE MOUSE.
     * 
     * @return std::array<bool,3> : a boolean representing the walls to the 
     * current left, center, right
     */
    std::array<bool,3> sense() {
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
            int ri = rs[i] + mouse_r;
            int ci = cs[i] + mouse_c;
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
    void sense_update(std::array<bool,3> w) {
        mem_maze[index(mouse_r,mouse_c)] = 0;
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
    ipair floodfill() {
        int target_r;
        int target_c;

        vector<pair<int, int>> unvisited;
        vector<pair<int, int>> cur_layer = {{mouse_r, mouse_c}};

        while (unvisited.empty()) {
            for (auto a : get_adj({r, c})) {

                // Add visited to queue
                if (mem_maze[index(a.first, a.second)] == VISITED) {
                    
                }
            }
        }

    }

    /**
     * @brief Makes mouse travel from its current position (defined in class vars
     * r and c) to the dest, scanning walls while doing so. Note that the mouse
     * can only travel on visited nodes to get there
     * 
     * @param dest 
     */
    int moveto(ipair dest) {
        int tr = dest.first, tc = dest.second;
        vector<ipair> path = shortest_path(mouse_r, mouse_c, tr, tc);
        
        // If there is no valid path error out
        if (path.empty()) return 0;

        for (ipair node : path) {

            auto walls = sense();
            sense_update(walls);
            
            // navigate to next position
            int nr = node.first, nc = node.second;

            // determine orentation
            if (nr > mouse_r) {
                dir = 3; // up
            } if (nr < mouse_r) {
                dir = 1;
            } if (nc > mouse_c) {
                dir = 0;
            } if (nc < mouse_c) {
                dir = 2;
            }

            // move mouse
            mouse_r = nr;
            mouse_c = nc;
        }

        return 1;
    }
};


int main() {
    Navigator n;

    n.print_real_maze();
    
    for (auto p : path) {
        auto next = p;
        
        n.print_mem_maze();
    }
    

/*
    for (auto adj : n.get_adj({0, 0}))
        printf("(%d, %d), ", adj.first, adj.second);
    printf("\n");

    for (int i = 0; i < 7; i ++) {
        for (int j = 0; j < 8; j++) {
            printf("%d ", n.memH_maze[index(i,j)]);
        }
        printf("\n");
    }

    printf("\n");

    for (int i = 0; i < 7; i ++) {
        for (int j = 0; j < 8; j++) {
            printf("%d ", n.realH_maze[index(i,j)]);
        }
        printf("\n");
    }
*/


    n.print_mem_maze();

}