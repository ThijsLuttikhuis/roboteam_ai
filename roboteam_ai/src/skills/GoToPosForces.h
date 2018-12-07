//
// Created by rolf on 03/12/18.
//

#ifndef ROBOTEAM_AI_GOTOPOSFORCES_H
#define ROBOTEAM_AI_GOTOPOSFORCES_H
#include "Skill.h"
#include "../interface/Interface.h"

namespace rtt{
namespace ai{
class GoToPosForces : public Skill {
    private:
        using status = bt::Node::Status;
        roboteam_msgs::WorldRobot robot;
        roboteam_msgs::World world;
        enum Progression {
          ON_THE_WAY, DONE, FAIL
        };
        Progression currentProgress;
        void checkProgression();
        Vector2 targetPos;
        Vector2 deltaPos;
        bool isCollision(Vector2 usPos, Vector2 usVel, Vector2 objPos, Vector2 objVel, double time);
        double searchDir(double angleOffset);
        void sendMoveCommand();

        int lowestcollisioncount;
        double bestangle;
        bool collision;
        std::vector<Vector2> displayData;
        interface::Interface interface;

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
