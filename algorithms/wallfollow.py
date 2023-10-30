from api import *
import sys

def log(string):
    sys.stderr.write("{}\n".format(string))
    sys.stderr.flush()

def main():
    log("Running left wall algorithm...")
    setColor(0, 0, "G")
    setText(0, 0, "abc")
    while True:
        if not wallLeft():
            turnLeft()
        while wallFront():
            turnRight()
        moveForward()

if __name__ == "__main__":
    main()