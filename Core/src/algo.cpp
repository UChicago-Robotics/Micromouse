#include "algo.h"
#include "Arduino.h"
#include "comm.h"
#include <algorithm>
#include <iostream>
#include <queue>
#include <unordered_set>
#include <vector>

// #include "API.h"
using namespace std;
typedef pair<int, int> ii;


String printWall(Wall w) {
    return "Wall: L: " + String(w.left) + ", F: " + String(w.front) + ", R: " + String(w.right);
}

String printInstruction(Instruction d) {
    String s = "";
    switch (d) {
        case DRIVE_STRAIGHT:
            s = "DRIVE_STRAIGHT";
            break;
        case TURN_LEFT:
            s = "TURN_LEFT";
            break;
        case TURN_RIGHT:
            s = "TURN_RIGHT";
            break;
        case TURN_AROUND:
            s = "TURN_AROUND";
    }
    return "Instruction: " + s;
}

char orientation_to_char(Orientation o) {
    if (o == NORTH) return 'n';
    if (o == WEST) return 'w';
    if (o == SOUTH) return 's';
    if (o == EAST) return 'e';
}

int index(int ri, int ci) {
    return ri * MAZE_SIZE + ci;
}

ii getrc(int index) {
    return {(int)(index / MAZE_SIZE), index % MAZE_SIZE};
}

int d2c(int index) {
    int r = getrc(index).first, c = getrc(index).second;
    int h = MAZE_SIZE / 2;
    return min(abs(r - h - 1), abs(r - h)) + min(abs(c - h - 1), abs(c - h));
}

int l1(int index) {
    int r = getrc(index).first, c = getrc(index).second;
    return r + c;
}


bool in_bounds(int r, int c) {
    return (0 <= r && 0 <= c && r < MAZE_SIZE && c < MAZE_SIZE);
}

Orientation add_orientation(Orientation o, int offset) {
    int res = (o + offset + MOD_ORIENTATION) % MOD_ORIENTATION;
    return static_cast<Orientation>(res);
}

Orientation left_of(Orientation o) {
    return add_orientation(o, 1);
}

Orientation right_of(Orientation o) {
    return add_orientation(o, -1);
}


Navigator::Navigator() {
    // North, West, South, East
    // 1 is wall, 0 is no wall, -1 is no information
    knowledge.resize(MAZE_SIZE, vector<vector<int>>(MAZE_SIZE, vector<int>(4, -1)));
    seen.resize(N, false);
    flood.resize(N, 0);
    adjSet.resize(MAZE_SIZE * MAZE_SIZE);

    // further initialize knowledge with what's known
    for (int c = 0; c < MAZE_SIZE; ++c) {
        knowledge[0][c][SOUTH] = 1;     // bottom row has SOUTH border
        knowledge[MAXR][c][NORTH] = 1;  // top row has NORTH border
    }
    for (int r = 0; r < MAZE_SIZE; ++r) {
        knowledge[r][0][WEST] = 1;     // leftmost column has WEST border
        knowledge[r][MAXC][EAST] = 1;  // rightmost column has EAST border
    }

    // initialize adjList
    // 2 cells are adjacent if there are no walls or we don't know if there are walls between them
    // this adjList encodes the same information as knowledge, just in a more convenient form
    for (int r = 0; r < MAZE_SIZE; ++r) {
        for (int c = 0; c < MAZE_SIZE; ++c) {
            if (r < MAXR) adjSet[index(r, c)].insert(index(r + 1, c));
            if (c > 0) adjSet[index(r, c)].insert(index(r, c - 1));
            if (r > 0) adjSet[index(r, c)].insert(index(r - 1, c));
            if (c < MAXC) adjSet[index(r, c)].insert(index(r, c + 1));
        }
    }
}

void Navigator::newLoop() {
    seen[index(mouse_r, mouse_c)] = true;
    seenVertices.push(index(mouse_r, mouse_c));
    printstr("Current pos: mouse_r: " +  String(mouse_r) + ", mouse_c: " + String(mouse_c) + ", orientation: " + orientation_to_char(orientation));
}

Instruction Navigator::giveInstruction() {
    if (justTurned) {
        // possibly check if front wall is there, just to doublecheck
        return DRIVE_STRAIGHT;
    }
    int targv = 0, targr = 0, targc = 0, currmin = INF;

    // this variable and some part of this procedure is only for RETRACE mode
    int count = 0;
    bool chosen = false;
    while (count < 2 && !chosen) {
        for (auto v : adjSet[index(mouse_r, mouse_c)]) {
            // the first time, if in RETRACE mode, want to find vertices
            // that we havent seen before.
            // the variable chosen is to ensure that if we run through this once
            // and everything is already seen, we still gotta move on
            // update: this is still quite naive, because even if practically
            // all the edges around the cell are already figured out
            // the algorithm would try to go there anyway
            if ((mode == RETRACE || mode == RERETRACE) && seen[v] && count == 0) continue;
            
            if (flood[v] < currmin) {
                // L1 tiebreaker here? I tried, somewhat tricky but doable
                targv = v;
                targr = getrc(v).first;
                targc = getrc(v).second;
                currmin = flood[v];
                chosen = true;
            }
        }
        ++count;
    }

    // cerr << "targr :" << targr << " targc: " << targc << " flood: " << currmin << endl;
    Orientation targO;
    if (targr > mouse_r) targO = NORTH;

    if (targr < mouse_r) targO = SOUTH;

    if (targc < mouse_c) targO = WEST;

    if (targc > mouse_c) targO = EAST;

    // if at correct orientation, then drive straight
    // else, turn the correct way
    // note left is positive
    // cerr << "targO: " << orientation_to_char(targO) << " curr orientation: " << orientation_to_char(orientation) << endl;

    // warning! don't change the logic here
    // funky
    if (targO == orientation) return DRIVE_STRAIGHT;
    if (targO == add_orientation(orientation, 1)) return TURN_LEFT;
    if (targO == add_orientation(orientation, 2)) return TURN_AROUND;
    return TURN_RIGHT;
}

void Navigator::reflood() {
    vector<int> dist(N, INF);
    priority_queue<ii, vector<ii>, std::greater<ii>> pq;

    if (mode == SCAN || mode == RERETRACE) {
        // fill 0 in the center squares
        for (int r = MAXR / 2; r <= MAXR / 2 + 1; ++r) {
            for (int c = MAXC / 2; c <= MAXC / 2 + 1; ++c) {
                dist[index(r, c)] = 0;
                pq.push(make_pair(0, index(r, c)));
            }
        }
    } else {
        // mode == RETRACE
        dist[index(0, 0)] = 0;
        pq.push(make_pair(0, index(0, 0)));
    }

    while (!pq.empty()) {
        int u = pq.top().second;
        pq.pop();
        for (auto v : adjSet[u]) {
            if (dist[v] > dist[u] + 1) {
                dist[v] = dist[u] + 1;
                pq.push(make_pair(dist[v], v));
            }
        }
    }

    for (int r = 0; r < MAZE_SIZE; ++r) {
        for (int c = 0; c < MAZE_SIZE; ++c) {
            flood[index(r, c)] = dist[index(r, c)];
            // API::setText(c, r, to_string(flood[index(r, c)]));
        }
    }
}

void Navigator::performInstruction(Instruction instr) {
    // once the instruction was performed, update position
    // and any other thing if needed
    orientation = add_orientation(orientation, instr);
    // order should not matter here!
    if (instr == DRIVE_STRAIGHT) {
        switch (orientation) {
            case NORTH:
                ++mouse_r;
                break;
            case WEST:
                --mouse_c;
                break;
            case SOUTH:
                --mouse_r;
                break;
            case EAST:
                ++mouse_c;
                break;
        }
    }

    if (!in_bounds(mouse_r, mouse_c)) cout << "Out of bounds! BAD!" << endl;
}

void Navigator::setWall(int r, int c, Orientation o, int val) {
    // set value to the wall at orientation o of cell (r, c)
    knowledge[r][c][o] = val;
    // if (val == 1) API::setWall(c, r, orientation_to_char(o));
    // now, have to set neighboring cells as well
    switch (o) {
        case NORTH:
            if (r < MAXR) {
                knowledge[r + 1][c][SOUTH] = val;
                if (val == 1) {
                    adjSet[index(r, c)].erase(index(r + 1, c));
                    adjSet[index(r + 1, c)].erase(index(r, c));
                } else {
                    adjSet[index(r, c)].insert(index(r + 1, c));
                    adjSet[index(r + 1, c)].insert(index(r, c));
                }
            }
            break;
        case WEST:
            if (c > 0) {
                knowledge[r][c - 1][EAST] = val;
                if (val == 1) {
                    adjSet[index(r, c)].erase(index(r, c - 1));
                    adjSet[index(r, c - 1)].erase(index(r, c));
                } else{
                    adjSet[index(r, c)].insert(index(r, c - 1));
                    adjSet[index(r, c - 1)].insert(index(r, c));
                }
            }
            break;
        case SOUTH:
            if (r > 0) {
                knowledge[r - 1][c][NORTH] = val;
                if (val == 1) {
                    adjSet[index(r, c)].erase(index(r - 1, c));
                    adjSet[index(r - 1, c)].erase(index(r, c));
                } else {
                    adjSet[index(r, c)].insert(index(r - 1, c));
                    adjSet[index(r - 1, c)].insert(index(r, c));

                }
            }
            break;
        case EAST:
            if (c < MAXC) {
                knowledge[r][c + 1][WEST] = val;
                if (val == 1) {
                    adjSet[index(r, c)].erase(index(r, c) + 1);
                    adjSet[index(r, c + 1)].erase(index(r, c));
                } else {
                    adjSet[index(r, c)].insert(index(r, c) + 1);
                    adjSet[index(r, c + 1)].insert(index(r, c));
                }
            }
            break;
    }
}

// make sure to tell the Navigator what movement was previously performed
// so that the walls can be imported correctly
void Navigator::importWall(Wall w) {
    // using current position and orientation
    printstr("Importing walls: " + printWall(w));
    setWall(mouse_r, mouse_c, orientation, w.front);
    setWall(mouse_r, mouse_c, left_of(orientation), w.left);
    setWall(mouse_r, mouse_c, right_of(orientation), w.right);
}
void Navigator::calibrateMode() {
    if ((mode == SCAN || mode == RERETRACE) && mouse_r >= MID - 1 && mouse_r <= MID && mouse_c >= MID - 1 && mouse_c <= MID) mode = RETRACE;

    if (mode == RETRACE && mouse_r == 0 && mouse_c == 0) mode = RERETRACE;
    // this might look stupid, but we'll see, cus we'll alternate between scanning
    // and retracing, but hopefully we'll find more routes on the way
}

void Navigator::deleteInfo(int nSteps) {
    printstr("Deleting info for " + String(nSteps) + " most recent cells");
    int counter = 0;
    while(counter < nSteps && !this->seenVertices.empty()) {
        int topCell = seenVertices.top();
        int ri = getrc(topCell).first;
        int ci = getrc(topCell).second;
        setWall(ri, ci, NORTH, -1);
        setWall(ri, ci, WEST, -1);
        setWall(ri, ci, SOUTH, -1);
        setWall(ri, ci, EAST, -1);
        seenVertices.pop();
        ++counter;
    }
}