from api import *
import sys
import random

def log(string):
    sys.stderr.write("{}\n".format(string))
    sys.stderr.flush()

"""
Basic DFS. Not a particularly good algorithm, but works as a springboard
for other algorithms you may want to write.
"""


def get_left_dir(cur_dir: str) -> str:
    return {"N": "W", "W": "S", "S": "E", "E": "N"}[cur_dir]

def get_right_dir(cur_dir) -> str:
    return {"N": "E", "W": "N", "S": "W", "E": "S"}[cur_dir]

def get_opp_dir(cur_dir) -> str:
    return {"N": "S", "W": "E", "S": "N", "E": "W"}[cur_dir]

def update_coords(cc: tuple, move_dir: str) -> tuple:
    if move_dir == "N": return (cc[0], cc[1] + 1)
    if move_dir == "S": return (cc[0], cc[1] - 1)
    if move_dir == "E": return (cc[0] + 1, cc[1])
    if move_dir == "W": return (cc[0] - 1, cc[1])

def turn_dir(cur_dir: str, next_dir: str):
    if next_dir == get_right_dir(cur_dir): turnRight90()
    if next_dir == get_left_dir(cur_dir): turnLeft90()
    if next_dir == get_opp_dir(cur_dir): turn180()


def dfs():
    pivot_points = {} # nodes in our graph
    cur_pos = (0, 0)
    cur_dir = "N"
    
    
    while True:
        next_move = ""

        # check possible next moves
        possible_moves = []
        if not wallFront(): 
            possible_moves += cur_dir
        if not wallRight(): 
            possible_moves += get_right_dir(cur_dir)
        if not wallLeft(): 
            possible_moves += get_left_dir(cur_dir)
        
        log(f"{cur_dir}, {possible_moves}, {cur_pos}")

        # if there are no possible moves, turn around
        if len(possible_moves) == 0:
            next_move = get_opp_dir(cur_dir)

        # If there is 1 possible move, just head in that direction
        elif len(possible_moves) == 1:
            next_move = possible_moves[0]
    
        # if there are > 1 possible moves, we are at a pivot, choose leftmost
        # and save the current position
        else:
            # if pivot is visited
            if cur_pos in pivot_points.keys():
                
                # node has available moves
                if pivot_points[cur_pos]:
                    next_move = random.choice(pivot_points[cur_pos])
                    pivot_points[cur_pos].remove(next_move)
                # node is exhausted
                else:
                    next_move = get_opp_dir(cur_dir)
            
            # if pivot is unvisited
            else:
                pivot_points[cur_pos] = possible_moves
                next_move = random.choice(pivot_points[cur_pos])
                pivot_points[cur_pos].remove(next_move)

        # orient to the correct position and move
        turn_dir(cur_dir, next_move)
        cur_pos = update_coords(cur_pos, cur_dir)
        cur_dir = next_move

        moveForward()


def retrace_to_pivot(pivot_queue: list, prev_moves: list) -> int:
    pass

def main():
    log("Running DFS algorithm...") 
    dfs()

if __name__ == "__main__":
    main()