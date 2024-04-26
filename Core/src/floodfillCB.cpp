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

int index(int r, int c) {
    return r * MAZE_SIZE + c;
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
                    std::cout << ".";
                    break;
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
    int h = MAZE_SIZE/2;
    return min(abs(r-h-1),abs(r-h)) + min(abs(c-h-1),abs(c-h));
}

uint8_t* memV_maze = (uint8_t*) calloc(sizeof(char), MAZE_SIZE*MAZE_SIZE-1);
uint8_t* memH_maze = (uint8_t*) calloc(sizeof(char), MAZE_SIZE*MAZE_SIZE-1);
uint8_t* mem_maze = (uint8_t*) calloc(sizeof(char), MAZE_SIZE*MAZE_SIZE);

uint8_t* realV_maze = (uint8_t*) calloc(sizeof(char), MAZE_SIZE*MAZE_SIZE-1);
uint8_t* realH_maze = (uint8_t*) calloc(sizeof(char), MAZE_SIZE*MAZE_SIZE-1);
uint8_t* real_maze = (uint8_t*) calloc(sizeof(char), MAZE_SIZE*MAZE_SIZE);

uint8_t HW[7][8] = {
    {1,1,1,1,0,1,0,1},
    {1,0,1,1,0,0,1,1},
    {0,0,1,1,1,1,0,0},
    {0,0,0,0,0,0,1,0},
    {0,0,0,1,1,0,0,0},
    {0,0,0,1,1,1,0,0},
    {0,0,1,0,1,0,0,0}
};
uint8_t VW[8][7] = {
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
void navigate(std::vector<int> nav) { // nav is vector of -1,0,1,2 for left+straight straight right+straight, 2 turn around + drive
    // TODO arduino implementation as actual driving
    
    // change internal position coords
    for (int i : nav) {
        dir = (dir + 4 + i) % 4;
        if (dir % 2 == 0) {
            c += 1 - dir;
        } else {
            r += 2 - dir;
        }
    };
}

std::vector<int> floodfill(std::array<bool,3> visuals) {
  std::vector<int> nav = {0,0,0}; // placeholder
  //TODO WRITE FLOODFILL ALGORITHM
  return nav;
}


std::array<bool,3> sense() {
    // emulator for the mouse sensing the physical walls

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



int main() {
    //<example maze>
    // declare arrs
    for (int i = 0; i < MAZE_SIZE; i++) {
        for (int j = 0; j < MAZE_SIZE; j++) {
            mem_maze[index(i, j)] = 0;
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
    //<\example maze>


    // <examples of subroutines navigate, sense, done>
    // driving navigation demos
    std::vector<int> nav = {0,0,0,1,0,1,-1};
    navigate(nav); // (0,0) -> (3,2)
    std::cout << r << " " << c << std::endl;
    nav = {2,0};
    navigate(nav); // (3,2) -> (1,2)
    std::cout << r << " " << c << std::endl;

    // sensing walls
    r = 4;
    c = 5;
    dir = 0;
    std::array<bool,3> s = sense();
    std::cout << s[0] << " " << s[1] << " " << s[2] << std::endl;

    // done
    r = 3;
    c = 5;
    std::cout << done() << std::endl;
    // <\examples of subroutines navigate, sense, done>



    // <pseudo algorithm>
    while (!done()) {
      std::array<bool,3> walls = sense();
      std::vector<int> nav = floodfill(walls); // all other variables necessary for algo are global
      navigate(nav);
    }
    // <\pseudo algorithm>
}