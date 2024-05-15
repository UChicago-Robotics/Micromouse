# Setup

1. [Download the Micromouse simulator](https://github.com/mackorone/mms#download)
2. Run the simulator and click the "+" button to configure a new algorithm
3. Enter the config for your algorithm (name, directory, build command, and run command)

In particular, for our (UChicago MM) purposes:
- name = whatever you want
- directory = the folder this file is in, i.e., `AlgoSim`
- build command = `g++ -o a.out -std=c++11 API.cpp algo.cpp main.cpp`
- run command = `./a.out`

If I'm not wrong the last command (running the file `./a.out`) might differ on macOS vs Windows; the big idea is that you run it, so run it.

4. Choose a maze by browsing from the `mazes` folder here.
4. Click Build and then Run.

# File Structure
- `main.cpp` emulates `Core/main.cpp`
- `algo.cpp` is where the action is. Should be able to directly import that into `Core/`, with the removal of the `mms` API calls.
- all printouts are through `cerr` in `algo.cpp`, since the framework uses `stdin/stdout` for the scripts to interact. `cerr` outputs can be seen in the application window itself.
- `API.cpp` and `API.h` are files for the simulation framework. Don't touch.
- `floodfill.cpp` is Gideon's `floodfillFINAL.cpp` (commit `0e10702`, approx 9pm May 15). 

# Idea structure
- I hope everything is generally clear.
- For the flooding algorithm, I used Dijkstra with a `priority_queue` to do the heavy lifting.

# Interacting with the Sim
- Read the docs. it's pretty clear cut.


