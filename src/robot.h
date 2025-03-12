#ifndef __MY_ROBOT_AGENT__H
#define __MY_ROBOT_AGENT__H 

#include "enviro.h"
#include <queue>

using namespace enviro;

const double MOVE_SPEED = 10;

class MyRobotController : public Process, public AgentInterface {
public:
    MyRobotController() : Process(), AgentInterface(), movingToGoal(false), targetX(0), targetY(0) {}

    void init() {
        watch("path_found", [this](const Event& e) {
            queuePathPoints(e.value());
            startMovingIfNeeded();
        });
    }
    
    void start() {}

    void update() {
        if (movingToGoal) {
            if (isAtTarget()) {
                reachCurrentTarget();
            } else {
                move_toward(targetX, targetY, MOVE_SPEED);
            }
        }
    }
    
    void stop() {}

private:
    std::queue<std::pair<double, double>> path;
    bool movingToGoal;
    double targetX, targetY;

    void queuePathPoints(const json& pathData) {
        for (auto& point : pathData) {
            path.push({point["x"], point["y"]});
        }
    }
    
    void startMovingIfNeeded() {
        if (!path.empty() && !movingToGoal) {
            moveToNextPoint();
        }
    }

    void moveToNextPoint() {
        if (!path.empty()) {
            std::pair<double, double> p = path.front();
            targetX = p.first;
            targetY = p.second;
            movingToGoal = true;
        }
    }
    
    double distanceToTarget() {
        return sqrt(pow(targetX - position().x, 2) + pow(targetY - position().y, 2));
    }
    
    bool isAtTarget() {
        return distanceToTarget() <= MOVE_SPEED / 2;
    }
    
    void reachCurrentTarget() {
        path.pop();
        if (!path.empty()) {
            moveToNextPoint();
        } else {
            movingToGoal = false;
            track_velocity(0, 0);
        }
    }
};

class MyRobot : public Agent {
public:
    MyRobot(json spec, World& world) : Agent(spec, world) {
        add_process(c);
    }
private:
    MyRobotController c;
};

DECLARE_INTERFACE(MyRobot)

#endif