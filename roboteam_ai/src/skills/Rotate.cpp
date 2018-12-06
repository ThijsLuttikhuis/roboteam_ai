//
// Created by thijs on 24-10-18.
//

#include "Rotate.h"
#include "math.h"

namespace rtt {
namespace ai {

Rotate::Rotate(string name, bt::Blackboard::Ptr blackboard)
        :Skill(name, blackboard) {
}

/// Init the Rotate skill
void Rotate::initialize() {

    if (properties->hasString("ROLE")) {
        std::string roleName = properties->getString("ROLE");
        robot.id = (unsigned int) dealer::findRobotForRole(roleName);
        if (World::getRobotForId(robot.id, true)) {
            robot = World::getRobotForId(robot.id, true).get();
        }
        else {
            ROS_ERROR("Rotate Initialize -> robot does not exist in world");
            currentProgress = Progression::FAIL;
            return;
        }
    }
    else {
        ROS_ERROR("Rotate Initialize -> ROLE WAITING!!");
        currentProgress = Progression::FAIL;
        return;
    }

//  ____________________________________________________

    rotateToBall = properties->getBool("rotateToBall");
    rotateToOurGoal = properties->getBool("rotateToOurGoal");
    rotateToEnemyGoal = properties->getBool("rotateToEnemyGoal");
    rotateToRobotID = properties->getInt("rotateToRobotID");
    robotIsEnemy = properties->getBool("robotIsEnemy");

    if (properties->hasDouble("Angle")) {
        targetAngle = properties->getDouble("Angle");
    }
//    else {
//        ROS_ERROR("Rotate Initialize -> No good angle set in properties");
//        currentProgress = Progression::FAIL;
//    }

}

bt::Node::Status Rotate::update() {

    if (World::getRobotForId(robot.id, true)) {
        robot = World::getRobotForId(robot.id, true).get();
    }
    else {
        ROS_ERROR("Rotate Update -> robot does not exist in world");
        currentProgress = Progression::FAIL;
    }

    if (rotateToBall) {
        auto ball = World::getBall();
        Vector2 deltaPos = {ball.pos.x - robot.pos.x, ball.pos.y - robot.pos.y};
        targetAngle = deltaPos.angle();

    }
    else if (rotateToEnemyGoal) {
        auto enemyGoal = Field::get_their_goal_center();
        Vector2 deltaPos = {enemyGoal.x - robot.pos.x, enemyGoal.y - robot.pos.y};
        targetAngle = deltaPos.angle();

    }
    else if (rotateToEnemyGoal) {
        auto ourGoal = Field::get_their_goal_center();
        Vector2 deltaPos = {ourGoal.x - robot.pos.x, ourGoal.y - robot.pos.y};
        targetAngle = deltaPos.angle();

    }
    else if (rotateToRobotID != - 1) {
        if (robotIsEnemy) {
            if (World::getRobotForId(rotateToRobotID, false)) {
                auto otherRobot = World::getRobotForId(rotateToRobotID, false).get();
                Vector2 deltaPos = {otherRobot.pos.x - robot.pos.x, otherRobot.pos.y - robot.pos.y};
                targetAngle = deltaPos.angle();

            }
        }
        else {
            if (World::getRobotForId(rotateToRobotID, true)) {
                auto otherRobot = World::getRobotForId(rotateToRobotID, true).get();
                Vector2 deltaPos = {otherRobot.pos.x - robot.pos.x, otherRobot.pos.y - robot.pos.y};
                targetAngle = deltaPos.angle();
            }
        }
    }

    roboteam_msgs::RobotCommand command;
    command.id = robot.id;
    command.use_angle = 1;

    command.w = (float) Control::calculateAngularVelocity(robot.angle, targetAngle);
    publishRobotCommand(command);
    currentProgress = checkProgression();

    switch (currentProgress) {
    case ROTATING: return Status::Running;
    case DONE: return Status::Success;
    case FAIL: return Status::Failure;
    }

    return Status::Failure;
}

void Rotate::terminate(Status s) {
    roboteam_msgs::RobotCommand command;
    command.id = robot.id;
    command.use_angle = 1;
    command.w = 0.0f;
    publishRobotCommand(command);
}

std::string Rotate::node_name() {
    return "Rotate";
}

Rotate::Progression Rotate::checkProgression() {

    double errorMargin = 0.05;
    if (deltaAngle > errorMargin) return ROTATING;
    else return DONE;
}

} // ai
} // rtt
