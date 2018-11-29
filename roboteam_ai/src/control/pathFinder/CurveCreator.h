//
// Created by simen on 06/11/18.
//

#ifndef ROBOTEAM_ai_CURVECREATOR_H
#define ROBOTEAM_ai_CURVECREATOR_H

#include <vector>
#include <roboteam_utils/Vector2.h>
#include "armadillo"
#include "../../utilities/World.h"

namespace rtt {
    namespace ai {
        class CurveCreator {
        private:
            // Variables
            float robotDiameter = 0.18; // TODO: Should be pulled from some list with constants
            float maxVelocity = 1.0; // maximum absolute velocity TODO: Should be pulled from some list with constants
            std::vector<Vector2> curveAccelerations;
            std::vector<Vector2> curveVelocities;
            std::vector<float> curveOrientations;
            std::vector<Vector2> curvePositions;
            std::vector<Vector2> controlPoints;
            float numPoints; // number of curve points
            double totalTime;

                // Functions
            std::vector<Vector2> findDangerousObstacle(std::vector<Vector2> trianglePoints,
                    std::vector<Vector2> robotCoordinates);
            float distancePointToLine(Vector2 point, Vector2 linepoint1, Vector2 linepoint2);
            Vector2 pointOnLinePastObstacle(Vector2 startPoint, Vector2 obstaclePos, std::vector<Vector2> linepoints);
            void calculateControlPoints(std::vector<Vector2> pathNodes, std::vector<Vector2> &robotCoordinates, float startVelocity, float endVelocity);
            void convertPointsToCurve();
            void calculateVelocity();
            void calculateAcceleration();
            void calculateOrientation();
            double factorial(float x);
            void addVelocityControlPoints(float startVelocity, float endVelocity);
            std::vector<Vector2> createConvexHull();
            bool isObstacleInConvexHull(std::vector<Vector2> polygon, Vector2 obstPos);

        public:
            CurveCreator();
            CurveCreator(float numPoints);
            void createCurve(std::vector<Vector2> pathNodes, std::vector<Vector2> robotCoordinates, float startVelocity, float endVelocity);

            double getTotalTime() const;

            const std::vector<Vector2> &getCurvePositions() const;

            const std::vector<Vector2> &getCurveVelocities() const;

            const std::vector<float> &getCurveOrientations() const;
        };

    } // ai
} // rtt

#endif //ROBOTEAM_ai_CURVECREATOR_H
