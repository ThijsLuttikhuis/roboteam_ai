#include <utility>

//
// Created by baris on 29-11-18.
//

#include "DefaultTactic.h"

// ------ EDIT -----

bt::Node::Status bt::DefaultTactic::update() {
    if (claimedRobots != robotsNeeded) {
        claimRobots();
        status = Status::Waiting;
    }
    else {
        auto status = child->tick();

        if (status == Status::Success) {
            return Status::Success;
        }

        else {
            return Status::Running;
        }
    }
    return status;
}


// ---- DO NOT EDIT ----

bt::DefaultTactic::DefaultTactic(std::string name, bt::Blackboard::Ptr blackboard,
        std::map<std::string, robotType> robots_) {

    robots = std::move(robots_);
    globalBB = std::move(blackboard);
    setName(std::move(name));
}
void bt::DefaultTactic::setName(std::string newName) {
    name = std::move(newName);
}

void bt::DefaultTactic::initialize() {
    claimRobots();
}

void bt::DefaultTactic::terminate(bt::Node::Status s) {
    dealer::removeTactic("victoryDanceTactic");

    child->terminate(child->getStatus());

    if (s == Status::Running) {
        setStatus(Status::Failure);
    }
    claimedRobots = 0;
}

void bt::DefaultTactic::claimRobots() {

    for (const auto &role : roleNames) {
        robotIDs.insert(dealer::claimRobotForTactic(robotType::random, name, role));
        if (robotIDs.find(- 1) == robotIDs.end()) claimedRobots ++;
        else robotIDs.erase(- 1);
    }
}

std::string bt::DefaultTactic::node_name() {
    return name;
}



