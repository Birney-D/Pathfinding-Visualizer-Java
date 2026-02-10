# Pathfinding & Maze Generation Visualizer

A Java GUI application that visualizes classic pathfinding algorithms and maze generation in real time.

This project demonstrates how algorithms such as **A\***, **BFS**, and **DFS** explore a grid to find optimal paths, along with **Wilson’s Algorithm** for generating perfect mazes. Built as an educational and interactive way to understand how these algorithms behave step-by-step.

## Features

- Real-time visualization of:
  - A* (A Star)
  - Breadth-First Search (BFS)
  - Depth-First Search (DFS)
- Maze generation using **Wilson’s Algorithm**
- Adjustable grid size
- Auto Start/End node placement
- Option to manually build walls or place obstacles with mouse
- Step-by-step algorithm animation with adjustable speed
- GUI built with Java (AWT/Swing)

## Technologies

- Java
- Maven
- AWT / Swing for GUI

## How to Run

### Compile
```bash
mvn clean package
```

### Run
* args[0] = grid width (1 - 100)
* args[2] = grid height (1 - 100)
* args[2] = obstacle placement (auto or manual):
	- auto - maze generated automatically using Wilson's Algorithm
	- manual - use left mouse click to place blocks, right click to remove them
* args[3] = pathfinding algorithm choice (DFS, BFS, ASTAR)
* args[4] = pathfinding speed (1-3)

```bash
java -jar target/<your-jar-name>.jar <gridwidth> <gridheight> <manual/auto wall placement> <PFAlgorithm> <speed 1-3>
```
* Example java -jar path/to/jar.jar 20 20 auto ASTAR 2

