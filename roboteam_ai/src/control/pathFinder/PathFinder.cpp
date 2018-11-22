//
// Created by selina on 10/31/18.
//

#include "PathFinder.h"

// TODO - Make objectCoordinates matrix
// TODO - Make variables for start/end orientation
// TODO - Generate end position

namespace rtt{
    namespace ai{
        PathFinder::PathFinder() {

        }

        void PathFinder::calculatePath(Vector2 endPosition, Vector2 startPosition, float endAngle, float startAngle, float startVelocity,
                float endVelocity, std::vector<Vector2> robotCoordinates) {

            // Set start & end ID: always these values
            int startID = 0;
            int endID = 1;

            // Add start & end position to objects
            std::vector<Vector2> objectCoordinatesVector;
            objectCoordinatesVector.emplace_back(startPosition);
            objectCoordinatesVector.emplace_back(endPosition);

            // Add safety coordinates
            float safetyMargin = 0.3; // TODO from parameter list; distance between field and field border
            int nSteps = 5; // determines amount of safety points

            float fieldWidth = Field::get_field().field_width; // returns 0
            float fieldLength = Field::get_field().field_length; // returns 0

            std::vector<float> xEdges = {-fieldWidth/2 - safetyMargin, fieldWidth/2 + safetyMargin};
            for (float x: xEdges) {
                for (float y=-fieldLength/2; y<=fieldLength/2; y=y+fieldLength/nSteps) {
                    objectCoordinatesVector.emplace_back(Vector2(x,y));
                }
            }

            std::vector<float> yEdges = {-fieldLength/2 - safetyMargin, fieldLength/2 + safetyMargin};
            for (float y: yEdges) {
                for (float x=-fieldWidth/2; x<=fieldWidth/2; x=x+fieldWidth/nSteps) {
                    objectCoordinatesVector.emplace_back(Vector2(x,y));
                }
            }

            // Add robot coordinates
            objectCoordinatesVector.insert(objectCoordinatesVector.end(), robotCoordinates.begin(), robotCoordinates.end());

            // Change object vector to matrix
            arma::Mat<float> temp;
            arma::Mat<float> objectCoordinatesMatrix;
            for (int i = objectCoordinatesVector.size()-1; i > -1; i--) {
                temp << objectCoordinatesVector[i].x << objectCoordinatesVector[i].y << arma::endr;
                objectCoordinatesMatrix.insert_rows(0, temp);
            }

            VoronoiCreator voronoiCreator;
            VoronoiCreator::parameters voronoiParameters = voronoiCreator.createVoronoi(objectCoordinatesMatrix,
                    startAngle, endAngle);

            interface::Interface gui;
            gui.drawFrame(voronoiParameters.nodes, voronoiParameters.segments);

            FindShortestPath shortestPathFinder;
            std::vector<Vector2> path = shortestPathFinder.calculateShortestPath(voronoiParameters.nodes,
                    voronoiParameters.segments, startID, endID, objectCoordinatesVector, startAngle, endAngle);

            CurveCreator curveCreator;
            curveCreator.createCurve(path, objectCoordinatesVector, startVelocity, endVelocity);

            curvePoints = curveCreator.getCurvePositions();
            velocities = curveCreator.getCurveVelocities();
            angles = curveCreator.getCurveOrientations();
            totalTime = curveCreator.getTotalTime();
        }

        std::vector<Vector2> PathFinder::getCurvePoints() {
            return curvePoints;
        }

        std::vector<Vector2> PathFinder::getVelocities() {
            return velocities;
        }

        std::vector<float> PathFinder::getAngles() {
            return angles;
        }
    double PathFinder::getTotalTime() const {
        return totalTime;
    }
    }
}



