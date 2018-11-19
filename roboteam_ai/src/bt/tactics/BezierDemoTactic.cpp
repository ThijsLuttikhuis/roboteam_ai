
//
// Created by baris on 05/11/18.
//

#include <utility>
#include "BezierDemoTactic.h"
#include "../../utilities/World.h"

namespace bt {

BezierDemoTactic::BezierDemoTactic(std::string name, Blackboard::Ptr blackboard) {

    globalBB = std::move(blackboard);
    setName(std::move(name));
}

void BezierDemoTactic::setName(std::string newName) {
    name = std::move(newName);
}

void BezierDemoTactic::Initialize() {

    while (!claimedRobots) {
        std::set<int> ids;
        ids = RobotDealer::getAvailableRobots();
        if (!ids.empty()) {
            auto id = *ids.begin();  // only one robot..
            std::string roleName = "testRole";
            std::pair<int, std::string> idName = {id, roleName};
            claimedRobots = RobotDealer::claimRobotForTactic(idName, "bezierTactic");
            robotIDs.insert(id);
        }
    }
}

Node::Status BezierDemoTactic::Update() {
    auto status = child->Tick();

    if (status == Status::Success) {
        return Status::Success;
    }
    else if (status == Status::Invalid) {
        return Status::Failure;
    }
    else /* if (status == Status::Failure || status == Status::Running) */ {
        // If the status was anything but success/invalid, keep running
        return Status::Running;
    }
}
std::string BezierDemoTactic::node_name() {
    return "Bezier Demo Tactic";
}

} // bt










