//
// Created by rolf on 03/12/18.
//

#include "goToPosForces.h"
namespace rtt {
namespace ai {
GoToPosForces::GoToPosForces(rtt::string name, bt::Blackboard::Ptr blackboard)
        :Skill(name, blackboard) { }
std::string GoToPosForces::node_name() {
    return "GoToPosForces";
}

GoToPosForces::Progression GoToPosForces::checkProgression() {

}

void GoToPosForces::initialize() {
    if (properties->hasString("ROLE")) {
        std::string roleName = properties->getString("ROLE");
        robot.id = (unsigned int) dealer::findRobotForRole(roleName);
        if (World::getRobotForId(robot.id, true)) {
            robot = World::getRobotForId(robot.id, true).get();
        }
        else {
            ROS_ERROR("GoToPosForces Initialize -> robot does not exist in world");
            currentProgress = Progression::FAIL;
            return;
        }
    }
    else {
        ROS_ERROR("GoToPosForces Initialize -> ROLE WAITING!!");
        currentProgress = Progression::FAIL;
        return;
    }

    if (properties->hasVector2("Position")) {
        targetPos = properties->getVector2("Position");
    }
    else {
        ROS_ERROR("GoToPosForces Initialize -> No good X or Y set in properties");
        currentProgress = Progression::FAIL;
    }
}

bt::Node::Status GoToPosForces::update() {
    if (World::getRobotForId(robot.id, true)) {
        robot = World::getRobotForId(robot.id, true).get();
    }
    else {
        ROS_ERROR("GoToPosForces Update -> robot does not exist in world");
    }
    if (currentProgress == Progression::FAIL) {
        return Status::Failure;
    }
    //sendMoveCommand();
    currentProgress=checkProgression();
    switch (currentProgress) {
    case ON_THE_WAY:return Status::Running;
    case DONE: return Status::Success;
    case FAIL: return Status::Failure;
    }

    return Status::Failure;
}

void GoToPosForces::terminate(Status s) { }

bool GoToPosForces::isCollision(Vector2 usPos,Vector2 usVel,Vector2 objPos,Vector2 objVel,double time)
{
    Vector2 relVel=objVel-usVel;
    Vector2 closestPoint
}
}
}