//
// Created by baris on 05/11/18.
//
#include "../Tactic.h"
#include "../../../src/utilities/RobotDealer.h"

#ifndef ROBOTEAM_AI_BEZIERDEMOTACTIC_H
#define ROBOTEAM_AI_BEZIERDEMOTACTIC_H

namespace bt {

class BezierDemoTactic : public Tactic {

    public:
        BezierDemoTactic(std::string name, Blackboard::Ptr blackboard);

        std::string name;

        void setName(std::string newName);

        void Initialize() override;
        Node::Status Update() override;

        std::string node_name() override;

        bool claimedRobots = false;

        std::set<int> robotIDs = {};

//        Node::Ptr child = nullptr;



};

} // bt

#endif //ROBOTEAM_AI_TACTICNODE_H
