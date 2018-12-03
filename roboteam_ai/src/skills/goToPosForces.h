//
// Created by rolf on 03/12/18.
//

#ifndef ROBOTEAM_AI_GOTOPOSFORCES_H
#define ROBOTEAM_AI_GOTOPOSFORCES_H
#include "Skill.h"

namespace rtt{
namespace ai{
class GoToPosForces : public Skill {
    private:
        using status = bt::Node::Status;
        roboteam_msgs::WorldRobot robot;
        enum Progression {
          ON_THE_WAY, DONE, FAIL
        };
        Progression currentProgress;
        Progression checkProgression();
        Vector2 targetPos;
    public:
        explicit GoToPosForces(string name, bt::Blackboard::Ptr blackboard);
        std::string node_name() override;
        void initialize() override;
        Status update() override;
        void terminate(Status s) override;
};
}
}


#endif //ROBOTEAM_AI_GOTOPOSFORCES_H
