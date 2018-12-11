//
// Created by mrlukasbos on 23-10-18.
//

#ifndef ROBOTEAM_AI_SHOOT_H
#define ROBOTEAM_AI_SHOOT_H

#include "Skill.h"
#include "../utilities/World.h"

namespace rtt {
namespace ai {

class ShootAtGoal : public Skill {
    private:
        using status = bt::Node::Status;
        int amountOfCycles{};
    protected:
        virtual void sendKickCommand(double kickVel);
        enum Progression {
          KICKING, DONE, FAIL
        };
        Progression currentProgress;

    public:
        explicit ShootAtGoal(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
        Status update() override;
        void initialize() override;
};

} // ai
} // rtt

#endif //ROBOTEAM_AI_SHOOT_H
