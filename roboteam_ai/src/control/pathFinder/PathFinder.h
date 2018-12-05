//
// Created by selina on 10/31/18.
//

#ifndef ROBOTEAM_ai_PATHFINDER_H
#define ROBOTEAM_ai_PATHFINDER_H

#include <vector>
#include <roboteam_utils/Vector2.h>
#include "../../utilities/World.h"
#include "VoronoiCreator.h"
#include "FindShortestPath.h"
#include "CurveCreator.h"
#include "../../interface/Interface.h"

namespace rtt {
namespace ai {
class PathFinder {
    public:
        PathFinder();
        std::vector<Vector2> getPath();
        std::vector<Vector2> getCurvePoints();
        std::vector<Vector2> getVelocities();
        std::vector<float> getAngles();
        double getTotalTime() const;

        void calculatePath(Vector2 endPosition, Vector2 startPosition, float endAngle, float startAngle,
                float startVelocity,
                float endVelocity, std::vector<Vector2> robotCoordinates);

    private:
        interface::Interface gui;

        std::vector<Vector2> path;
        std::vector<Vector2> curvePoints;
        std::vector<Vector2> velocities;
        std::vector<float> angles;
        double totalTime;
        std::vector<Vector2> removeIfOutsideSquare(std::vector<Vector2> objectCoordinatesVector,
                float objectMargin, float fieldWidth, float fieldLength, Vector2 startPosition, Vector2 endPosition);
};
}
}

#endif //ROBOTEAM_ai_PATHFINDER_H
