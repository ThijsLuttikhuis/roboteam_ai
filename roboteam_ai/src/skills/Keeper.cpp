//
// Created by rolf on 10/12/18.
//

#include <roboteam_ai/src/interface/drawer.h>
#include <roboteam_ai/src/control/PID.h>
#include "Keeper.h"
namespace rtt {
namespace ai {
Keeper::Keeper(rtt::string name, bt::Blackboard::Ptr blackboard)
        :Skill(name, blackboard) { }
std::string Keeper::node_name() { return "Keeper"; }

void Keeper::initialize() {
    robot = getRobotFromProperties(properties);

    goalPos = Field::get_our_goal_center();
    goalwidth = Field::get_field().goal_width;

    //Create arc for keeper to drive on
    double diff = constants::KEEPER_POST_MARGIN - constants::KEEPER_CENTREGOAL_MARGIN;

    double radius = diff*0.5 + goalwidth*goalwidth/(8*diff);
    double angle = asin(goalwidth/2/radius);
    Vector2 center = Vector2(goalPos.x + constants::KEEPER_CENTREGOAL_MARGIN + radius, 0);
    if (diff>0) {
        blockCircle = Arc(center, radius, M_PI - angle, angle - M_PI);
    }
    else {
        blockCircle = Arc(center, radius,angle,-angle); //we take the radius from the other side so we have to also flip the arc (yes, confusing)
    }
    pid.setParams(3.3,0.0,100.0,10,0.0,0.0); //magic numbers galore, from the old team.
    pid.initialize(constants::tickRate);
}
Keeper::Status Keeper::update() {
    updateRobot();
    if (robot) {
        Vector2 ballPos = World::getBall().pos;
        Vector2 blockPoint = computeBlockPoint(ballPos);

        sendMoveCommand(blockPoint);
        return Status::Running;
    }
    else {
        return Status::Failure;
    }
}

void Keeper::terminate(Status s) {
    roboteam_msgs::RobotCommand cmd;
    cmd.use_angle = 1;
    cmd.id = robotId;
    cmd.x_vel = 0;
    cmd.y_vel = 0;
    cmd.w = static_cast<float>(M_PI_2);
    publishRobotCommand(cmd);

}

void Keeper::sendMoveCommand(Vector2 pos) {
//    double gain=3.0;
//    Vector2 delta = (pos - robot->pos)*gain; //TODO: add proper position control.
    Vector2 delta=pid.posControl(robot->pos,pos);
    roboteam_msgs::RobotCommand cmd;
    cmd.use_angle = 1;
    cmd.id = robot->id;
    cmd.x_vel = static_cast<float>(delta.x);
    cmd.y_vel = static_cast<float>(delta.y);
    cmd.w = static_cast<float>(M_PI_2);
    publishRobotCommand(cmd);
}
Vector2 Keeper::computeBlockPoint(Vector2 defendPos) {
    Vector2 u1 = (goalPos + Vector2(0.0, goalwidth*0.5) - defendPos).normalize();
    Vector2 u2 = (goalPos + Vector2(0.0, - goalwidth*0.5) - defendPos).normalize();
    double dist = (defendPos - goalPos).length();
    Vector2 blockLineStart = defendPos + (u1 + u2).stretchToLength(dist);
    std::pair<boost::optional<Vector2>, boost::optional<Vector2>> intersections = blockCircle.intersectionWithLine(
            blockLineStart, defendPos);
    Vector2 blockPos,posA,posB;
    // go stand on the intersection of the lines. Pick the one that is closest to (0,0) if there are multiple
    if (intersections.first&&intersections.second){
        posA=*intersections.first;
        posB=*intersections.second;
        if (posA.length()<posB.length()){
            blockPos=posA;
        }
        else blockPos=posB;
    }
    else if (intersections.first) {
        blockPos = *intersections.first;
    }
    else if (intersections.second) {
        blockPos = *intersections.second;
    }
    else {
        blockPos = Vector2(goalPos.x + constants::KEEPER_POST_MARGIN, goalwidth/2
                *signum(defendPos.y)); // Go stand at one of the poles depending on the side the defendPos is on.
    }
    return blockPos;
}
}
}