#include <algorithm>
#include <iostream>
#include <vector>
#include <unordered_set>
using namespace std;
const int MAZE_SIZE = 16;
const int MAXR = MAZE_SIZE - 1;
const int MAXC = MAZE_SIZE - 1;
const int N = MAZE_SIZE * MAZE_SIZE;
const int MID = MAZE_SIZE / 2;
const int INF = 10000;
const int MOD_ORIENTATION = 4;

enum Instruction {
    DRIVE_STRAIGHT = 0,
    TURN_LEFT = 1,
    TURN_RIGHT = -1
};

struct Wall {
    int left;
    int front;
    int right;
};


enum Orientation {
    NORTH = 0,
    WEST = 1,
    SOUTH = 2,
    EAST = 3
};

enum Mode {
    SCAN = 0,
    RETRACE = 1,
    RERETRACE = 2
};

void printWall(Wall w);
void printInstruction(Instruction d);

class Navigator {
   public:
    int mouse_r = 0;
    int mouse_c = 0;
    vector<bool> seen;
    vector<vector<vector<int> > > knowledge;
    vector<int> flood;
    vector<unordered_set<int> > adjSet;
    Orientation orientation = NORTH;
    bool justTurned = false;
    Mode mode = SCAN;
    Navigator();
    Instruction giveInstruction();
    void importWall(Wall w);
    void performInstruction(Instruction d);
    void setWall(int r, int c, Orientation o, int val);
    void reflood();
    void newLoop();
    void calibrateMode();
};