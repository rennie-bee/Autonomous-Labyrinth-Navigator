# Autonomous-Labyrinth-Navigator

## Overview
The Autonomous Labyrinth Navigator project presents an intriguing challenge in the field of autonomous navigation systems. This project involves designing and implementing a robotic system capable of navigating through a randomly generated maze to find an exit path. The maze is constructed with complexity and randomness in mind, featuring four potential exits—one on each side—ensuring multiple potential escape routes.

## Business Context
This project emerged from the growing need for advanced pathfinding algorithms in autonomous systems. Similar challenges are faced in warehouse robotics, where automated systems must efficiently navigate complex floor plans while avoiding obstacles. Companies like Warehouse Automation Solutions Inc. initially attempted to implement manual routing systems, but quickly discovered the limitations when dealing with dynamic environments where pathways might change due to inventory shifts or maintenance.

The Autonomous Labyrinth Navigator represents a scaled-down version of these real-world applications, allowing for the development and testing of pathfinding algorithms in a controlled environment before deployment in industrial settings. The ability to efficiently navigate through complex environments has significant implications for reducing operational costs and improving efficiency in various industries, from warehouse management to emergency response robots.

## Project Scope
The primary objectives of this project include:
- Maze Generation: Creating randomized yet solvable mazes using Eller's algorithm,
ensuring each maze has four potential exits (one on each side)
- Maze Construction: Developing a robust system for wall placement and cell
management within the maze environment.
- Path Optimization: Implementing algorithms to identify the shortest possible path from
the robot's starting position to any of the available exits.
- Robot Navigation: Designing the control mechanisms that will enable the robot to
follow the identified path and successfully exit the maze.

## Installation and Running the Code

### Prerequisites
- Docker.

### Steps
1. **Clone the Repository**:
   ```sh
   git clone https://github.com/rennie-bee/Autonomous-Labyrinth-Navigator
   ```
2. **Navigate to the project directory**:
    ```sh
    cd autonomous-labyrinth-navigator
    ```
3. **Run docker**:
   ```sh
   docker run -p80:80 -p8765:8765 -v $PWD:/source -it klavins/enviro:v1.6 bash
   ```
4. **Run Enviro**:
   ```sh
   make
   esm start
   enviro
   ```
5. Navigate to `http://localhost` to see the project.

<!-- 
## Using the Project
- On project launch, a maze will be generated with the robot positioned at the center.
- The robot will calculate and then follow the shortest path to the nearest exit.
- Run enviro again to generate a new maze and watch the robot solve it again. -->

## Acknowledgements
- Open-source [Enviro](https://github.com/klavinslab/enviro)  project provides the tools and frameworks utilized in this project.
