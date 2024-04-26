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
#include <iomanip>
#include <stdio.h>
#include <cmath>
#include <array>
#include <utility>
#include <queue>
#include <vector>
#include <unordered_set>
#include <stdint.h>
#include <stdexcept>
using namespace std;

//#define empty_f 0
#define WALL 1
#define EMPTY 0
#define UNVISITED 2
#define VISITED 3
#define MAZE_SIZE 8

int r = 0;
int c = 0;
int dir = 0; // 0 = right, 1 = down, 2 = left, 3 = up

int index(int ri, int ci) {
    return ri * MAZE_SIZE + ci;
}

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

int d2c(int r, int c) {
    // used to sort paths based on L1 norm of points
    int h = MAZE_SIZE/2;
    return min(abs(r-h-1),abs(r-h)) + min(abs(c-h-1),abs(c-h));
}

uint8_t* memV_maze = (uint8_t*) calloc(sizeof(char), MAZE_SIZE*MAZE_SIZE-1);
uint8_t* memH_maze = (uint8_t*) calloc(sizeof(char), MAZE_SIZE*MAZE_SIZE-1);
uint8_t* mem_maze = (uint8_t*) calloc(sizeof(char), MAZE_SIZE*MAZE_SIZE);

uint8_t* realV_maze = (uint8_t*) calloc(sizeof(char), MAZE_SIZE*MAZE_SIZE-1);
uint8_t* realH_maze = (uint8_t*) calloc(sizeof(char), MAZE_SIZE*MAZE_SIZE-1);
uint8_t* real_maze = (uint8_t*) calloc(sizeof(char), MAZE_SIZE*MAZE_SIZE);

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
 * @brief 
 * 
 * @param nav 
 */
void drivenav(std::vector<int> nav) { // nav is vector of -1,0,1,2 for left+straight straight right+straight, 2 turn around + drive
    // TODO arduino implementation as actual driving
    
    // change internal position coords r & c
    for (int i : nav) {
        dir = (dir + 4 + i) % 4;
        if (dir % 2 == 0) {
            c += 1 - dir;
        } else {
            r += 2 - dir;
        }
    };
}

/**
 * @brief returns the 2d-position from the 1d-index
 */
pair<int, int> get2dpos(int index) {
    return {(int) (index / MAZE_SIZE), index % MAZE_SIZE};
}

/**
 * @brief returns true if 2 cells are adjacent and the connection between 
 * them is known, else false
 * 
 * @param i1 
 * @param i2 
 * @return true 
 * @return false 
 */
bool connected(int i1, int i2) {
    int r1 = i1/MAZE_SIZE;
    int c1 = i1%MAZE_SIZE;
    int r2 = i2/MAZE_SIZE;
    int c2 = i2%MAZE_SIZE;
    if (abs(r1-r2) + abs(c1-c2) > 1) {return false;}
    return !((abs(r1-r2) > 0) ? memH_maze[index(min(r1,r2),c1)] : memV_maze[index(r1,min(c1,c2))]);
}

/**
 * @brief returns the min distance
 * 
 * @param dist 
 * @param sptSet 
 * @return int 
 */
int minDistance(const std::vector<int>& dist, const std::vector<bool>& sptSet) {
    int min = 257, min_index;
    for (int v = 0; v < MAZE_SIZE*MAZE_SIZE; ++v) {
        if (mem_maze[v] == UNVISITED) {
            continue;
        }
        if (!sptSet[v] && dist[v] <= min) {
            min = dist[v];
            min_index = v;
        }
    }
    return min_index;
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
 * of 1d-indices
 * 
 * @param r1 
 * @param c1 
 * @param r2 
 * @param c2 
 * @return std::vector<int> 
 */
std::vector<int> dijkstra(int r1, int c1, int r2, int c2) {
    int src = index(r1,c1);
    int dest = index(r2,c2);
    std::vector<int> dist(MAZE_SIZE*MAZE_SIZE, 257);
    std::vector<bool> sptSet(MAZE_SIZE*MAZE_SIZE, false);
    std::vector<int> parent(MAZE_SIZE*MAZE_SIZE, -1);
    dist[src] = 0;
    for (int count = 0; count < MAZE_SIZE*MAZE_SIZE - 1; ++count) {
        int u = minDistance(dist, sptSet);
        sptSet[u] = true;
        for (int v = 0; v < MAZE_SIZE*MAZE_SIZE; ++v) {
            if (mem_maze[v] == UNVISITED) {
                continue;
            }
            if (!sptSet[v] && connected(u, v) && dist[u] != 257 && dist[u] + connected(u, v) < dist[v]) {
                dist[v] = dist[u] + connected(u, v);
                parent[v] = u;
            }
        }
    }

    return getPath(parent, dest);
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
 * that are equidistant, 
 * If there's a fork, choose the node that's closer to the center.
 * 
 * @param visuals 
 * @return std::vector<int> 
 */
std::vector<int> floodfill() {
  int target_r; 
  int target_c;


    std::array<bool,3> walls = sense(); // read walls around
    sense_update(walls); // update memory to account for new walls
    std::vector<int> nav = floodfill(walls); // returns navigation to next cell
            // could be adjacent or could use dijkstra to retrace
 

  return path2nav(dijkstra(r,c,target_r,target_c));
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
void sense_update(std::array<bool,3> w) {
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
    //<example maze>
    // declare arrs
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


    // r and c are global variables corresponding to which cell the mouse is in
    // <pseudo algorithm>
    // while (!done()) { // until in center 4 squares
    //   std::array<bool,3> walls = sense(); // read walls around
    //   sense_update(walls); // update memory to account for new walls
    //   std::vector<int> nav = floodfill(walls); // returns navigation to next cell
            // could be adjacent or could use dijkstra to retrace
    //   drivenav(nav);
    // }
    // <\pseudo algorithm>
}