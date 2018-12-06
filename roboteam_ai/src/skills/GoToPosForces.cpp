//
// Created by rolf on 03/12/18.
//

#include "GoToPosForces.h"
namespace rtt {
namespace ai {
GoToPosForces::GoToPosForces(rtt::string name, bt::Blackboard::Ptr blackboard)
        :Skill(name, blackboard) { }
std::string GoToPosForces::node_name() {
    return "GoToPosForces";
}

void GoToPosForces::checkProgression() {
    if (currentProgress==ON_THE_WAY){
        if(deltaPos.length()<0.05){
            currentProgress=DONE;
        }
    }
    else if (currentProgress==DONE){return;}
    else if(currentProgress==FAIL){return;}

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
    currentProgress=ON_THE_WAY;
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
    world = World::get_world();
    deltaPos=targetPos-robot.pos;
    checkProgression();
    if (currentProgress==ON_THE_WAY){
        sendMoveCommand();
    }
    switch (currentProgress) {
    case ON_THE_WAY:return Status::Running;
    case DONE: return Status::Success;
    case FAIL: return Status::Failure;
    }

    return Status::Failure;
}

void GoToPosForces::terminate(Status s) { }

bool GoToPosForces::isCollision(Vector2 usPos, Vector2 usVel, Vector2 objPos, Vector2 objVel, double time) {
    Vector2 relVel = objVel - usVel;
    Vector2 predObjPos = objPos + relVel*time;
    double closestApproachDist = control::ControlUtils::distanceToLineWithEnds(usPos, objPos, predObjPos);
    return closestApproachDist <= constants::COLLISION_DISTANCE;
}

double GoToPosForces::searchDir(double angleOffset) {
    if (angleOffset < - M_PI || angleOffset > M_PI) { return 0; }
    int collisioncount=0;
    for (auto TheirBot: world.them) {
        if (isCollision(robot.pos, robot.vel, TheirBot.pos, TheirBot.vel, constants::LOOKAHEAD_TIME)) {
            collisioncount++;
            if (collisioncount>lowestcollisioncount){break;}
        }

    }
    if (collisioncount<=lowestcollisioncount) {
        for (auto OurBot: world.us) {
            if (OurBot.id != robot.id) {
                if (isCollision(robot.pos, robot.vel, OurBot.pos, OurBot.vel, constants::LOOKAHEAD_TIME)) {
                    collisioncount ++;
                    if (collisioncount > lowestcollisioncount) { break; }
                }
            }
        }
    }
    if (collisioncount==0) {
        return angleOffset;
    }
    else {
        if ((collisioncount<lowestcollisioncount||(collisioncount==lowestcollisioncount&&(fabs(angleOffset)<fabs(bestangle))))){
            bestangle=angleOffset;
            lowestcollisioncount=collisioncount;
        }
        if (angleOffset == 0) {
            double left=searchDir(angleOffset-0.01*M_PI);
            double right=searchDir(angleOffset+0.01*M_PI);
            if(left==0){
                if(right==0){
                    return bestangle;
                }
                else return right;
            }
            if(right==0) return left;
            if (fabs(left)<fabs(right)){
                return left;
            }
            else return right;
        }
        else if (angleOffset < 0) {
            return searchDir( angleOffset - 0.01*M_PI);
        }
        else return searchDir(angleOffset + 0.01*M_PI);
    }

}
void GoToPosForces::sendMoveCommand() {
    //initialize these 2 to be high numbers
    lowestcollisioncount=100;
    bestangle=1000;
    double angleOffset=searchDir(0); // We start by searching forwards
    Vector2 moveCmdDir=(deltaPos.rotate(angleOffset)).normalize();

    roboteam_msgs::RobotCommand cmd;
    cmd.id=robot.id;
    cmd.use_angle=1;
    cmd.x_vel= static_cast<float>(moveCmdDir.x*1.5);
    cmd.y_vel= static_cast<float>(moveCmdDir.y*1.5);
    cmd.w= static_cast<float>(moveCmdDir.angle());
    publishRobotCommand(cmd);
};

}
}