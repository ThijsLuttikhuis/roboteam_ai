//
// Created by selina on 11/14/18.
//

#ifndef ROBOTEAM_AI_GOTOPOSBEZIER_H
#define ROBOTEAM_AI_GOTOPOSBEZIER_H

#include "Skill.h"
#include "roboteam_utils/Vector2.h"
#include "../control/pathFinder/PathFinder.h"

namespace rtt {
namespace ai {

class GoToPosBezier : public Skill {
        using status = bt::Node::Status;

    public:
        Status Update() override;

        void Initialize() override;
        void Terminate(status s) override;
        explicit GoToPosBezier(string name, bt::Blackboard::Ptr blackboard);

        std::string node_name() override;

    private:

        // Variables
        time_t startTime;
        time_t now;
        double timeDif;
        double totalTime = 1; // TODO: get this value from PathFinder
        float angularVelocity;
        double xVelocity;
        double yVelocity;

        struct Curve {
          std::vector<rtt::Vector2> positions;
          std::vector<rtt::Vector2> velocities;
          std::vector<float> angles;
        };
        Curve curve;

        bool goToBall;

        roboteam_msgs::WorldRobot robot;
        int robotID;

        enum Progression {
          ON_THE_WAY, DONE, FAIL, INVALID
        };
        Progression currentProgress;
        Vector2 targetPos;

        bool checkTargetPos(Vector2 pos);

        void sendMoveCommand(float angularVelocity, double xVelocity, double yVelocity);

        Progression checkProgression();

        bool commandSend;

};
} // ai
} // rtt

#endif //ROBOTEAM_AI_GOTOPOSBEZIER_H
