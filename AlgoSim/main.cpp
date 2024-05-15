#include <iostream>
#include <string>

#include "API.h"
#include "algo.h"

void log(const std::string& text) {
    std::cerr << text << std::endl;
}



int main(int argc, char* argv[]) {
    log("Running...");
    Navigator nav;
    do {
        // init-like, but i don't wanna call it init
        // do things when starting a new loop
        nav.newLoop();
        // read walls
        nav.importWall({API::wallLeft(), API::wallFront(), API::wallRight()});
        // refloodfill
        nav.reflood();
        // get new instruction
        Instruction instr = nav.giveInstruction();
        cerr << "Received: ";
        printInstruction(instr);
        // physically perform instruction
        switch (instr) {
            case DRIVE_STRAIGHT: API::moveForward(); break;
            case TURN_LEFT: API::turnLeft(); break;
            case TURN_RIGHT: API::turnRight(); break;
        }
        // maybe conditioned on physical instruction being realized
        // but for now, assume physical instruction is achieved
        // so update nav system
        nav.performInstruction(instr);
        // check if arrived, to switch to RETRACE
        nav.calibrateMode();
    } while(true);
}
