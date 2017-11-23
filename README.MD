# A\*
This repo provides a complete C++ implementation of the A\* algorithm for pathfinding on a 2-D space.

## Usage
1. Install `opencv` first. 
2. Run the following to compile

```
g++ main.cpp -o astar `pkg-config --cflags opencv` `pkg-config --libs opencv`
```
3. Run `./astar`.

## References
1. [Introduction to A* from Red Blob Games](https://www.redblobgames.com/pathfinding/a-star/introduction.html)
2. [A very simple A* implementation in C++ callable from Python for pathfinding on a two-dimensional grid.](https://github.com/hjweide/a-star)

