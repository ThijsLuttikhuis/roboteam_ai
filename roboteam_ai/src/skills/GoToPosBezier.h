//
// Created by selina on 11/14/18.
//

#ifndef ROBOTEAM_AI_GOTOPOSBEZIER_H
#define ROBOTEAM_AI_GOTOPOSBEZIER_H

#include "Skill.h"
#include "roboteam_utils/Vector2.h"
#include "../control/pathFinder/PathFinder.h"
#include "../control/ControlUtils.h"

namespace rtt {
namespace ai {

class GoToPosBezier : public Skill {
        using status = bt::Node::Status;

    public:
        Status update() override;

        void initialize() override;
        void terminate(status s) override;
        explicit GoToPosBezier(string name, bt::Blackboard::Ptr blackboard);

        std::string node_name() override;

    private:

        // Variables
        std::chrono::system_clock::time_point startTime;
        std::chrono::system_clock::time_point now;
        std::chrono::duration<double> timeDif;
        float totalTime;
        control::ControlUtils::PIDvariables pidVarsInitial, pidVarsXPos, pidVarsYPos;
        PathFinder pathFinder;
        clock_t pidStartTime, pidEndTime;

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

        void sendMoveCommand(float desiredAngle, double xVelocity, double yVelocity);

        Progression checkProgression();

        bool commandSend;

        void updateCurveData(int currentPoint, bool isErrorTooLarge);

        bool isAnyObstacleAtCurve(int currentPoint);

};
} // ai
} // rtt

#endif //ROBOTEAM_AI_GOTOPOSBEZIER_H
