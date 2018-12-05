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

    updateCurveData(0, false);
    /// Set PID values
    pidVarsInitial.kP = 30.0;
    pidVarsInitial.kI = 0.1;
    pidVarsInitial.kD = 0.1;
    pidVarsInitial.prev_err = 0;

    pidVarsXPos.kP = pidVarsInitial.kP;
    pidVarsXPos.kI = pidVarsInitial.kI;
    pidVarsXPos.kD = pidVarsInitial.kD;
    pidVarsXPos.prev_err = pidVarsInitial.prev_err;

    pidVarsYPos.kP = pidVarsInitial.kP;
    pidVarsYPos.kI = pidVarsInitial.kI;
    pidVarsYPos.kD = pidVarsInitial.kD;
    pidVarsYPos.prev_err = pidVarsInitial.prev_err;

    pidStartTime = clock();
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

    // Calculate additional velocity due to position error
    Vector2 posError = curve.positions[currentPoint] - robot.pos;
    std::cout << "posError >> x: " << posError.x << "  y: " << posError.y << std::endl;

    pidEndTime = clock();
    pidVarsXPos.timeDiff = ((float)(pidEndTime-pidStartTime))/CLOCKS_PER_SEC;
    pidVarsYPos.timeDiff = ((float)(pidEndTime-pidStartTime))/CLOCKS_PER_SEC;
    pidStartTime = clock();
    //std::cout << "time diff: " << K.timeDiff << std::endl;
    //K.timeDiff = 0.016;
    float xOutputPID = control::ControlUtils::PIDcontroller((float)posError.x, pidVarsXPos);
    float yOutputPID = control::ControlUtils::PIDcontroller((float)posError.y, pidVarsYPos);

    //std::cout << "x out PID: " << xOutputPID << std::endl;

    curve.velocities[currentPoint].x += xOutputPID;
    curve.velocities[currentPoint].y += yOutputPID;

    auto desiredAngle = curve.angles[currentPoint];
    //std::cout << "ANGLE >> desired: " << desiredAngle << "  current: " << robot.angle <<  "  diff: " << desiredAngle - robot.angle << std::endl;
    // For old grSim:
//    double currentAngle = robot.angle;
//    double xVelocity = curve.velocities[currentPoint].x * cos(currentAngle) + curve.velocities[currentPoint].y * sin(currentAngle);
//    double yVelocity = -curve.velocities[currentPoint].x * sin(currentAngle) + curve.velocities[currentPoint].y * cos(currentAngle);
    double xVelocity = curve.velocities[currentPoint].x;
    double yVelocity = curve.velocities[currentPoint].y;

    // Send a move command
    sendMoveCommand(desiredAngle, xVelocity, yVelocity);

    // Determine if new curve is needed
    bool isAtEnd = currentPoint >= curve.positions.size() - 1;
    bool isErrorTooLarge = posError.length() > 1;
    bool hasTargetChanged = pathFinder.getPath().back().dist(targetPos) > 0.2;

    // Calculate new curve if needed
    if (isAtEnd || isErrorTooLarge || hasTargetChanged || isAnyObstacleAtCurve(currentPoint)) {
        if (isErrorTooLarge) {
            sendMoveCommand(robot.angle, 0, 0);
        }
        //clock_t begin = clock();
        updateCurveData(currentPoint, isErrorTooLarge);
        //clock_t end = clock();
        //std::cout << "seconds to calculate new curve: " << (double)(end - begin)/CLOCKS_PER_SEC << std::endl;

        std::cout << "------------------------" << std::endl;
        std::cout << "       NEW CURVE" << std::endl;
        std::cout << "------------------------" << std::endl;
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
void GoToPosBezier::sendMoveCommand(float desiredAngle, double xVelocity, double yVelocity) {
    if (! checkTargetPos(targetPos)) {
        ROS_ERROR("Target position is not correct GoToPos");
        return;
    }

    roboteam_msgs::RobotCommand command;
    command.id = robot.id;
    command.use_angle = 1;
    command.w = desiredAngle;
//    std::cout << "Current angle: " << robot.angle << std::endl;
//    std::cout << "Desired angle: " << desiredAngle << std::endl;

    command.x_vel = (float)xVelocity;
    command.y_vel = (float)yVelocity;

    publishRobotCommand(command);
    //std::cerr << "                  xvel: " << command.x_vel << ", yvel: " << command.y_vel << ", w_vel: " << command.w
    //          << std::endl;
}

/// Check the progress the robot made and alter the currentProgress
GoToPosBezier::Progression GoToPosBezier::checkProgression() {

    double dx = targetPos.x - robot.pos.x;
    double dy = targetPos.y - robot.pos.y;
    double deltaPos = (dx*dx) + (dy*dy);
    double maxMargin = 0.1;                 // max offset or something.

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
void GoToPosBezier::updateCurveData(int currentPoint, bool isErrorTooLarge) {
    // TODO: don't hardcode end orientation & velocity
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

    //float startAngle = robotVel.x == 0 ? robot.angle : (float)robotVel.angle(); // If robotVel in x-dir is 0, robotVel.angle() will be NaN
    auto endAngle = (float) M_PI;
    float endVelocity = 0;
    Vector2 robotVel = robot.vel;
    auto startVelocity = (float)robotVel.length();
    Vector2 startPos = robot.pos;
    float startAngle = robot.angle;

    if (!curve.positions.empty() && !isErrorTooLarge) {
        startPos = curve.positions[currentPoint];
        startAngle = curve.angles[currentPoint];
        startVelocity = (float)curve.velocities[currentPoint].length();
    }

    //startPos = startPos + robotVel.scale(0.08); // Add prediction based on delay
    startAngle < 0 ? startAngle = startAngle + 2*(float)M_PI : startAngle;
    endAngle < 0 ? endAngle = endAngle + 2*(float)M_PI : endAngle;

    pathFinder.calculatePath(targetPos, startPos, endAngle, startAngle, startVelocity, endVelocity, robotCoordinates);

    /// Get path parameters
    curve.positions = pathFinder.getCurvePoints();
    curve.velocities = pathFinder.getVelocities();
    curve.angles = pathFinder.getAngles();
    totalTime = pathFinder.getTotalTime();

    /// Start timer
    startTime = std::chrono::system_clock::now();
}

bool GoToPosBezier::isAnyObstacleAtCurve(int currentPoint) {
    double margin = 0.18;
    auto world = World::get_world();

    Vector2 robotPos;
    for (int i = currentPoint; i < curve.positions.size(); i++) {
        for (auto ourBot: world.us) {
            if (ourBot.id != robot.id) {
                robotPos = ourBot.pos;
                if (robotPos.dist(curve.positions[i]) < margin) {
                    return true;
                }
            }
        }
        for (auto theirBot: world.them) {
            robotPos = theirBot.pos;
            if (robotPos.dist(curve.positions[i]) < margin) {
                return true;
            }
        }
    }
    return false;
}

} // ai
} // rtt
