//
// Created by selina on 10/31/18.
//

#include "PathFinder.h"
#include <time.h>

// TODO - Make objectCoordinates matrix
// TODO - Make variables for start/end orientation
// TODO - Generate end position

namespace rtt {
namespace ai {
PathFinder::PathFinder() {
}

void PathFinder::calculatePath(Vector2 endPosition, Vector2 startPosition, float endAngle, float startAngle,
        float startVelocity,
        float endVelocity, std::vector<Vector2> robotCoordinates) {

    clock_t begin = clock();

    // Set start & end ID: always these values
    int startID = 0;
    int endID = 1;

    // Add start & end position to objects
    std::vector<Vector2> objectCoordinatesVector;
    objectCoordinatesVector.emplace_back(startPosition);
    objectCoordinatesVector.emplace_back(endPosition);

    // Add safety coordinates
    float safetyMargin = 0.1; // m TODO from parameter list; distance between field and field border
    int nSteps = 5; // determines amount of safety points

    float fieldWidth = Field::get_field().field_width;
    float fieldLength = Field::get_field().field_length;

//    float fieldWidth = (float) abs(startPosition.y - endPosition.y);
//    float fieldLength = (float) abs(startPosition.x - endPosition.x);

    std::vector<float> xEdges = {- fieldWidth/2 - safetyMargin, fieldWidth/2 + safetyMargin};
    for (float x: xEdges) {
        for (float y = - fieldLength/2; y <= fieldLength/2; y = y + fieldLength/nSteps) {
            objectCoordinatesVector.emplace_back(Vector2(x, y));
        }
    }

    std::vector<float> yEdges = {- fieldLength/2 - safetyMargin, fieldLength/2 + safetyMargin};
    for (float y: yEdges) {
        for (float x = - fieldWidth/2; x <= fieldWidth/2; x = x + fieldWidth/nSteps) {
            objectCoordinatesVector.emplace_back(Vector2(x, y));
        }
    }

    // Add robot coordinates
    objectCoordinatesVector.insert(objectCoordinatesVector.end(), robotCoordinates.begin(), robotCoordinates.end());

    // Remove objects that are outside of the mini-field + safety margin
//    float objectMargin = 1;
//    objectCoordinatesVector = removeIfOutsideSquare(objectCoordinatesVector, objectMargin, fieldWidth, fieldLength,
//            startPosition, endPosition);

    // Change object vector to matrix
    arma::Mat<float> temp;
    arma::Mat<float> objectCoordinatesMatrix;
    for (int i = objectCoordinatesVector.size() - 1; i > - 1; i --) {
        temp << objectCoordinatesVector[i].x << objectCoordinatesVector[i].y << arma::endr;
        objectCoordinatesMatrix.insert_rows(0, temp);
    }

    VoronoiCreator voronoiCreator;
    VoronoiCreator::parameters voronoiParameters = voronoiCreator.createVoronoi(objectCoordinatesMatrix,
            startAngle, endAngle);

    FindShortestPath shortestPathFinder;
    path = shortestPathFinder.calculateShortestPath(voronoiParameters.nodes,
            voronoiParameters.segments, startID, endID, objectCoordinatesVector, startAngle, endAngle);

    CurveCreator curveCreator;
    curveCreator.createCurve(path, objectCoordinatesVector, startVelocity, endVelocity);

    curvePoints = curveCreator.getCurvePositions();
    velocities = curveCreator.getCurveVelocities();
    angles = curveCreator.getCurveOrientations();
    totalTime = curveCreator.getTotalTime();

    gui.drawFrame(voronoiParameters.nodes, voronoiParameters.segments, curvePoints);

    clock_t end = clock();
    //std::cout << "seconds to end: " << (double)(end - begin)/CLOCKS_PER_SEC << std::endl;
}

std::vector<Vector2> PathFinder::getPath() {
    return path;
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
float PathFinder::getTotalTime() const {
    return totalTime;
}
std::vector<Vector2> PathFinder::removeIfOutsideSquare(std::vector<Vector2> objectCoordinatesVector,
        float objectMargin, float w, float l, Vector2 startPosition, Vector2 endPosition) {

    std::vector<Vector2> corners;

    // TODO: this could probably be better
    if (endPosition.x > startPosition.x && endPosition.y > startPosition.y) {
        corners.emplace_back(Vector2(endPosition.x + objectMargin, (endPosition.y + objectMargin))); // top right
        corners.emplace_back(Vector2(endPosition.x + objectMargin, endPosition.y - objectMargin - w)); // bottom right
        corners.emplace_back(Vector2(startPosition.x - objectMargin, startPosition.y + objectMargin + w)); // top left
        corners.emplace_back(Vector2(startPosition.x - objectMargin, startPosition.y - objectMargin)); // bottom left
    }
    else if (endPosition.x > startPosition.x && endPosition.y < startPosition.y) {
        corners.emplace_back(Vector2(endPosition.x + objectMargin, endPosition.y + objectMargin + w));
        corners.emplace_back(Vector2(endPosition.x + objectMargin, endPosition.y - objectMargin));
        corners.emplace_back(Vector2(startPosition.x - objectMargin, startPosition.y + objectMargin));
        corners.emplace_back(Vector2(startPosition.x - objectMargin, startPosition.y - objectMargin - w));
    }
    else if (endPosition.x < startPosition.x && endPosition.y < startPosition.y) {
        corners.emplace_back(Vector2(startPosition.x + objectMargin, startPosition.y + objectMargin));
        corners.emplace_back(Vector2(startPosition.x + objectMargin, startPosition.y - objectMargin - w));
        corners.emplace_back(Vector2(endPosition.x - objectMargin, endPosition.y + objectMargin + w));
        corners.emplace_back(Vector2(endPosition.x - objectMargin, endPosition.y - objectMargin));
    }
    else if (endPosition.x < startPosition.x && endPosition.y > startPosition.y) {
        corners.emplace_back(Vector2(startPosition.x + objectMargin, startPosition.y + objectMargin + w));
        corners.emplace_back(Vector2(startPosition.x + objectMargin, startPosition.y - objectMargin));
        corners.emplace_back(Vector2(endPosition.x - objectMargin, endPosition.y + objectMargin));
        corners.emplace_back(Vector2(endPosition.x - objectMargin, endPosition.y - objectMargin - w));
    }

    // Filter points that are outside of the square
    int i = 0;
    while (i < objectCoordinatesVector.size()) {
        if ((objectCoordinatesVector[i].x > corners[0].x || // x < top right x = valid
            objectCoordinatesVector[i].x < corners[2].x) || // x > top left x = valid
            (objectCoordinatesVector[i].y > corners[0].y || // y < top right y = valid
            objectCoordinatesVector[i].y < corners[1].y)) { // y > bottom right y = valid
            objectCoordinatesVector.erase(objectCoordinatesVector.begin() + i);
        }
        else {
            i ++;
        }
    }

    return objectCoordinatesVector;
}

} //ai
} //rtt



