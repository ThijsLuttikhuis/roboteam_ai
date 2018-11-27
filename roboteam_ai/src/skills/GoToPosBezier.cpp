//
// Created by selina on 11/14/18.
//

#include "GoToPosBezier.h"

namespace rtt {
namespace ai {

// TODO: Make if GrSim, if real robot statement

/// Init GoToPosBezier
void GoToPosBezier::Initialize() {

    if (properties->hasString("ROLE")) {
        std::string roleName = properties->getString("ROLE");
        robotID = (unsigned int) RobotDealer::findRobotForRole(roleName);
        if (World::getRobotForId(robotID, true)) {
            robot = World::getRobotForId(robotID, true).get();
        }
        else {
            ROS_ERROR("GoToPos Initialize -> robot does not exist in world");
            currentProgress = Progression::INVALID;
            return;
        }
    }
    else {
        ROS_ERROR("GoToPos Initialize -> ROLE INVALID!!");
        currentProgress = Progression::INVALID;
        return;
    }

    if (properties->hasBool("goToBall")) {
        goToBall = properties->getBool("goToBall");
    }
    else goToBall = false;

    if (goToBall) {
        auto ball = World::getBall();
        targetPos = ball.pos;
    }
    else {
        if (properties->hasVector2("Position")) {
            Vector2 posVector = properties->getVector2("Position");
            targetPos = posVector;
            currentProgress = Progression::ON_THE_WAY;

        }
        else {
            ROS_ERROR("GoToPos Initialize -> No good X, Y or ROBOT_ID set in BB, GoToPos");
            currentProgress = Progression::FAIL;
        }
    }

    updateCurveData();
}

/// Get an update on the skill
bt::Node::Status GoToPosBezier::Update() {

    if (World::getRobotForId(robotID, true)) {
        robot = World::getRobotForId(robotID, true).get();
    }
    else {
        ROS_ERROR("GoToPos Update -> robot does not exist in world");
        currentProgress = Progression::INVALID;
    }

    if (goToBall) {
        auto ball = World::getBall();
        targetPos = ball.pos;
    }

    // See if the progress is a failure
    if (currentProgress == Progression::FAIL) {
        return status::Failure;
    }
    else if (currentProgress == Progression::INVALID) {
        return status::Invalid;
    }

    // Check if the goal point on the curve is reached
    now = std::chrono::system_clock::now();
    timeDif = now - startTime;
    int currentPoint = (int) round((timeDif.count()/totalTime*curve.positions.size()));
    currentPoint = currentPoint >= (int) curve.positions.size() ? (int) curve.positions.size() - 1 : currentPoint;
    double currentAngle = robot.angle;

    // Calculate additional velocity due to position error
    Vector2 posError = curve.positions[currentPoint] - robot.pos;
    float xOutputPID = control::ControlUtils::PIDcontroller((float)posError.x, K);
    float yOutputPID = control::ControlUtils::PIDcontroller((float)posError.y, K);
    //float xOutputPID = 0; float yOutputPID = 0;

    // Set variables
    float angularVelocity = 0; //(curve.angles[currentPoint] - (float)currentAngle)/1000;
    double xVelocity = xOutputPID + curve.velocities[currentPoint].x * cos(currentAngle) + curve.velocities[currentPoint].y * sin(currentAngle);
    double yVelocity = yOutputPID + curve.velocities[currentPoint].x * sin(currentAngle) + curve.velocities[currentPoint].y * cos(currentAngle);

    // Send a move command
    sendMoveCommand(angularVelocity, xVelocity, yVelocity);


    // Calculate new curve if at the end
    if (currentPoint == curve.positions.size()-1) {
        //sendMoveCommand(0,0,0);
        updateCurveData();
    }

    // Now check the progress we made
    currentProgress = checkProgression();

    switch (currentProgress) {

        // Return the progression in terms of status
    case ON_THE_WAY:return status::Running;
    case DONE: return status::Success;
    case FAIL: return status::Failure;
    case INVALID: return status::Invalid;
    }

    return status::Failure;
}

/// Check if the vector is a valid one
bool GoToPosBezier::checkTargetPos(Vector2 pos) {
    // TODO: actually check
    return true;
}

/// Send a move robot command with a vector
void GoToPosBezier::sendMoveCommand(float angularVelocity, double xVelocity, double yVelocity) {
    if (! checkTargetPos(targetPos)) {
        ROS_ERROR("Target position is not correct GoToPos");
        return;
    }

    roboteam_msgs::RobotCommand command;
    command.id = robot.id;
    command.use_angle = 1;
    command.w = angularVelocity;

    command.x_vel = (float)xVelocity;
    command.y_vel = (float)yVelocity;

    publishRobotCommand(command);
    std::cerr << "                  xvel: " << command.x_vel << ", yvel: " << command.y_vel << ", w_vel: " << command.w
              << std::endl;
}

/// Check the progress the robot made and alter the currentProgress
GoToPosBezier::Progression GoToPosBezier::checkProgression() {

    double dx = targetPos.x - robot.pos.x;
    double dy = targetPos.y - robot.pos.y;
    double deltaPos = (dx*dx) + (dy*dy);
    double maxMargin = 0.2;                 // max offset or something.

    if (abs(deltaPos) >= maxMargin) return ON_THE_WAY;
    else return DONE;
}

GoToPosBezier::GoToPosBezier(string name, bt::Blackboard::Ptr blackboard)
        :Skill(name, blackboard) {

}

std::string GoToPosBezier::node_name() {
    return "GoToPosBezier";
}

void GoToPosBezier::Terminate(status s) {

    roboteam_msgs::RobotCommand command;
    command.id = robot.id;
    command.use_angle = 1;
    command.w = 0;

    command.x_vel = 0;
    command.y_vel = 0;

    publishRobotCommand(command);
}

/// Create robot coordinates vector & other parameters for the path
void GoToPosBezier::updateCurveData() {
    // TODO: don't hardcode end orientation & velocity
    auto endAngle = (float) M_PI;
    float endVelocity = 0;
    Vector2 robotVel = robot.vel;
    auto startVelocity = (float)robotVel.length();

    auto world = World::get_world();
    std::vector<Vector2> robotCoordinates;
    for (auto ourBot: world.us) {
        if (ourBot.id != robot.id) {
            robotCoordinates.emplace_back(ourBot.pos);
        }
    }
    for (auto theirBot: world.them) {
        robotCoordinates.emplace_back(theirBot.pos);
    }

    float startAngle;
    if (curve.positions.empty()) {
        startAngle = robot.angle;
    } else {
        startAngle = (float)curve.velocities.back().angle();
    }

    startAngle < 0 ? startAngle = startAngle + 2*(float)M_PI : startAngle = startAngle;
    endAngle < 0 ? endAngle = endAngle + 2*(float)M_PI : endAngle = endAngle;
    pathFinder.calculatePath(targetPos, robot.pos, endAngle, startAngle, startVelocity, endVelocity, robotCoordinates);

    /// Get path parameters
    curve.positions = pathFinder.getCurvePoints();
    curve.velocities = pathFinder.getVelocities();
    curve.angles = pathFinder.getAngles();
    totalTime = pathFinder.getTotalTime();

    /// Start timer
    startTime = std::chrono::system_clock::now();

    /// Set PID values
    K.prev_err = 0;
    K.kP = 0.01;
    K.kI = 0.2;
    K.kD = 0.01;
    K.timeDiff = 0.016; // 60 Hz?
}

} // ai
} // rtt
