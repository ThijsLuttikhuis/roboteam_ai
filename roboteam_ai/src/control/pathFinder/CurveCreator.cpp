//
// Created by simen on 06/11/18.
//

#include "CurveCreator.h"

namespace rtt {
namespace ai {
CurveCreator::CurveCreator() {
    this->numPoints = 1000;
}

CurveCreator::CurveCreator(float numPoints) {
    this->numPoints = numPoints;
}

void CurveCreator::createCurve(std::vector<Vector2> pathNodes, std::vector<Vector2> robotCoordinates,
        float startVelocity, float endVelocity) {
    robotCoordinates.erase(robotCoordinates.begin()); // Delete start point from objects
    robotCoordinates.erase(robotCoordinates.begin()); // Delete end point from objects
    if (pathNodes.size() < 2) {
        std::cout << "You need to enter at least 2 nodes in order to create a curve." << std::endl;
    }
    else {
        calculateControlPoints(pathNodes, robotCoordinates, startVelocity, endVelocity);
        convertPointsToCurve();
        calculateVelocity();
        calculateAcceleration();
        calculateOrientation();
    }
}

void CurveCreator::calculateControlPoints(std::vector<Vector2> pathNodes, std::vector<Vector2> &robotCoordinates,
        float startVelocity, float endVelocity) {
    controlPoints.push_back(pathNodes[0]); // First path node is always a control point
    controlPoints.push_back(pathNodes[1]); // Second path node is always a control point

    if (pathNodes.size() > 2) {
        controlPoints.push_back(pathNodes[2]); // Third path node is always a control point if it exists
        std::vector<Vector2> convex;
        std::vector<Vector2> dangerousObstacle;

        for (int i = 2; i < pathNodes.size() - 1; i ++) {
            convex = createConvexHull();
            dangerousObstacle = findDangerousObstacle(convex, robotCoordinates);
            if (dangerousObstacle.empty()) {
                controlPoints.push_back(pathNodes[i] + (pathNodes[i + 1] - pathNodes[i]).scale(
                        1)); // TODO: .scale() could be used to minimize curvature
            }
            else {
                // change control point until no obstacle is in the convex anymore
                // TODO: determine the control point in a mathematical way
                while (!dangerousObstacle.empty()) {
                    controlPoints[controlPoints.size()-1] = controlPoints[controlPoints.size()-2] +
                            (controlPoints[controlPoints.size()-1] - controlPoints[controlPoints.size()-2]).scale(0.5);
                    convex = createConvexHull();
                    dangerousObstacle = findDangerousObstacle(convex, robotCoordinates);
                }
                controlPoints[controlPoints.size()-1] = controlPoints[controlPoints.size()-2] +
                        (controlPoints[controlPoints.size()-1] - controlPoints[controlPoints.size()-2]).scale(1); // TODO: .scale() could be used to minimize curvature
                break;
            }
        }
    }

    bool isEndPiece = controlPoints.back() == pathNodes.back();
    addVelocityControlPoints(startVelocity, endVelocity, isEndPiece);

}

std::vector<Vector2> CurveCreator::findDangerousObstacle(std::vector<Vector2> convex,
        std::vector<Vector2> robotCoordinates) {
    std::vector<Vector2> dangerousObstacle;
    for (Vector2 &obstaclePos: robotCoordinates) {
        if (isObstacleInConvexHull(convex, obstaclePos)) {
            dangerousObstacle.push_back(obstaclePos);
            break;
        }
    }
    return dangerousObstacle;
}

std::vector<Vector2> CurveCreator::createConvexHull() {
    /// Use Graham Scan to turn curve piece into Convex Hull
    // Find point P that has the lowest y-coordinate (or x-coordinate if y is equal)
    Vector2 P(DBL_MAX, DBL_MAX);
    for (Vector2 &point : controlPoints) {
        if ((point.y < P.y) or (point.y == P.y and point.x < P.x)) {
            P = point;
        }
    }

    // Sort vector based on angle between P -> point and x-axis
    std::vector<Vector2> sortedList;
    sortedList.push_back(P);

    Vector2 unitVecX(1, 0);
    for (Vector2 &point : controlPoints) {
        if (point != P) {
            if (sortedList.size() == 1) {
                sortedList.push_back(point);
            }
            else {
                for (int i = 1; i < sortedList.size(); i ++) {
                    if ((point - P).normalize().dot(unitVecX) > (sortedList[i] - P).normalize().dot(unitVecX)) {
                        sortedList.insert(sortedList.begin() + i, point);
                        break;
                    } else if (i == sortedList.size()-1) {
                        sortedList.push_back(point);
                        break;
                    }
                }
            }
        }
    }

    // Iterate over points and check whether making a left or right turn
    std::vector<Vector2> convex;
    convex.push_back(sortedList[0]);
    convex.push_back(sortedList[1]);

    double crossProduct;
    int index = 0;
    Vector2 vec1, vec2;
    for (int i = 2; i < sortedList.size(); i++) {
        vec1 = convex[index + 1] - convex[index];
        vec2 = sortedList[i] - convex[index];
        crossProduct = vec1.x*vec2.y - vec1.y*vec2.x;

        if (crossProduct < 0) {
            // Right turn, erase last point from convex and add next one from list
            convex.pop_back();
            convex.push_back(sortedList[i]);
        }
        else {
            // Left turn, add point to convex
            convex.push_back(sortedList[i]);
            index ++;
        }
    }

    return convex;
}

bool CurveCreator::isObstacleInConvexHull(std::vector<Vector2> convex, Vector2 obstPos) {
    // Check if the obstacle is in the convex
    float sumOfAngles = 0;
    // If the sum of angles between obstacle to vertices is 2*PI, the obstacle is inside the polygon.
    // Simultaneously, check if the obstacle is on one of the edges.
    for (int i = 0; i < convex.size(); i ++) {
        if (i == convex.size() - 1) {
            sumOfAngles += acos((convex[i] - obstPos).normalize().dot((convex[0] - obstPos).normalize()));

            // Is obstacle on edge?
            if (distancePointToLine(obstPos, convex[i], convex[0]) < robotDiameter/2) {
                return true;
            }
        }
        else {
            sumOfAngles += acos((convex[i] - obstPos).normalize().dot((convex[i + 1] - obstPos).normalize()));

            // Is obstacle on edge?
            if (distancePointToLine(obstPos, convex[i], convex[i + 1]) < robotDiameter/2) {
                return true;
            }
        }
    }

    return abs(sumOfAngles - 2*M_PI) < 0.002*M_PI; // allow 0.1 percent deviation
}

float
CurveCreator::distancePointToLine(Vector2 point, Vector2 linepoint1, Vector2 linepoint2) {
    // use vector formulation
    // line: x = a + t*n, where a = linepoint1
    // point: p
    Vector2 n = (linepoint2 - linepoint1).normalize();
    double t = n.dot(point - linepoint1);
    if (t < 0) {
        // linepoint1 is closest to point
        return (float) point.dist(linepoint1);
    }
    else if (t > (linepoint2 - linepoint1).length()) {
        // linepoint 2 is closest to point
        return (float) point.dist(linepoint2);
    }
    else {
        // closest point is on the line segment
        return (float) (linepoint1 - point + n.stretchToLength(t)).length();
    }
}

Vector2 CurveCreator::pointOnLinePastObstacle(Vector2 startPoint, Vector2 obstaclePos,
        std::vector<Vector2> linepoints) {
    Vector2 pointOnLine;

    // Calculate direction vector from startPoint past the obstacle
    Vector2 startToObstacle = obstaclePos - startPoint;
    double rotationAngle = asin(0.5*robotDiameter/startToObstacle.length()); // Angle to rotate startToObstacle by
    Vector2 vectorPastObstacle = startToObstacle.rotate(rotationAngle);
    if (startToObstacle.dot(linepoints[0] - startPoint) > vectorPastObstacle.dot(linepoints[0] - startPoint)) {
        // The dot product with the first path line should become greater by rotating, otherwise rotate in the opposite direction
        vectorPastObstacle = startToObstacle.rotate(- rotationAngle);
    }

    // Calculate intersection between second path line and vectorPastObstacle
    Vector2 helpVector = Vector2(vectorPastObstacle.y/vectorPastObstacle.x, - 1);
    double scaleFactor = (startPoint.dot(helpVector) - linepoints[0].dot(helpVector))
            /(linepoints[1].dot(helpVector) - linepoints[0].dot(helpVector));
    if (isnan(scaleFactor) or scaleFactor > 1 or scaleFactor < 0) {
        scaleFactor = 0;
    }
    pointOnLine = linepoints[0] + (linepoints[1] - linepoints[0]).scale(scaleFactor);
    return pointOnLine;
}

void CurveCreator::addVelocityControlPoints(float startVelocity, float endVelocity, bool isEndPiece) {
    // add two control points to assure the given velocities
    startVelocity *= totalTime;
    endVelocity *= totalTime;
    auto numControlPoints = controlPoints.size() + 1; // Including velocity control points
    numControlPoints = isEndPiece ? numControlPoints + 1 : numControlPoints;

    float startScaleFactor = startVelocity/(numControlPoints - 1);
    startScaleFactor = startScaleFactor > (float) (controlPoints[1] - controlPoints[0]).length()
                       ? (float) (controlPoints[1] - controlPoints[0]).length() : startScaleFactor;
    Vector2 startVelCP = controlPoints[0] + (controlPoints[1] - controlPoints[0]).stretchToLength(startScaleFactor);
    controlPoints.insert(controlPoints.begin() + 1, startVelCP);

    if (isEndPiece) {
        float endScaleFactor = endVelocity/(numControlPoints - 1);
        endScaleFactor = endScaleFactor > (float) (controlPoints[controlPoints.size() - 2]
                - controlPoints[controlPoints.size() - 1]).length()
                         ? (float) (controlPoints[controlPoints.size() - 2]
                        - controlPoints[controlPoints.size() - 1]).length() : endScaleFactor;
        Vector2 endVelCP = controlPoints.back()
                + (controlPoints[controlPoints.size() - 2] - controlPoints[controlPoints.size() - 1]).stretchToLength(
                        endScaleFactor);
        controlPoints.insert(controlPoints.end() - 1, endVelCP);
    }
}

void CurveCreator::convertPointsToCurve() {
    float curveDegree = controlPoints.size() - 1;

    for (int i = 0; i < numPoints; i ++) {
        curvePositions.emplace_back(Vector2(0, 0));
    }

    double coefficient;
    double controlPointWeight;
    for (int i = 0; i <= curveDegree; i ++) {
        controlPointWeight = factorial(curveDegree)/(factorial(i)*factorial(curveDegree - i));
        float t = 0; // curve parameter
        for (int j = 0; j < numPoints; j ++) {
            coefficient = controlPointWeight*pow(t, i)*pow(1 - t, curveDegree - i);
            curvePositions[j] = curvePositions[j] + controlPoints[i].scale(coefficient);
            t += 1/numPoints;
        }
    }
}

void CurveCreator::calculateVelocity() {
    float curveDegree = controlPoints.size() - 1;

    for (int i = 0; i < numPoints; i ++) {
        curveVelocities.emplace_back(Vector2(0, 0));
    }

    double coefficient;
    double controlPointWeight;
    for (int i = 0; i <= curveDegree; i ++) {
        controlPointWeight = factorial(curveDegree)/(factorial(i)*factorial(curveDegree - i));
        float t = 0; // curve parameter
        for (int j = 0; j < numPoints; j ++) {
            coefficient = controlPointWeight*(i*pow(t, i - 1)*pow(1 - t, curveDegree - i)
                    - pow(t, i)*(curveDegree - i)*pow(1 - t, curveDegree - i - 1));
            curveVelocities[j] = curveVelocities[j] + controlPoints[i].scale(coefficient);
            t += 1/numPoints;
        }
    }

    // Fix NAN data by extrapolation
    curveVelocities[0] = curveVelocities[1] + (curveVelocities[1] - curveVelocities[2]);
    curveVelocities.back() = curveVelocities[curveVelocities.size() - 2]
            + (curveVelocities[curveVelocities.size() - 2] - curveVelocities[curveVelocities.size() - 3]);

    // Get the highest velocity that will be reached in the curve
    double highestVelocity = 0.0;
    for (const Vector2 &vel : curveVelocities) {
        highestVelocity = (vel.length() > highestVelocity) ? vel.length() : highestVelocity;
    }

    totalTime = highestVelocity/maxVelocity;
    for (int i = 0; i < numPoints; i ++) {
        curveVelocities[i].x /= totalTime;
        curveVelocities[i].y /= totalTime;
    }

    std::cout << "Highest curve velocity: " << highestVelocity << std::endl;
    std::cout << "Total time: " << totalTime << std::endl;
}

void CurveCreator::calculateAcceleration() {
    float curveDegree = controlPoints.size() - 1;

    for (int i = 0; i < numPoints; i ++) {
        curveAccelerations.emplace_back(Vector2(0, 0));
    }

    double coefficient;
    double controlPointWeight;
    for (int i = 0; i <= curveDegree; i ++) {
        controlPointWeight = factorial(curveDegree)/(factorial(i)*factorial(curveDegree - i));
        float t = 0; // curve parameter
        for (int j = 0; j < numPoints; j ++) {
            coefficient = controlPointWeight*(i*(i - 1)*pow(t, i - 2)*pow(1 - t, curveDegree - i)
                    - 2*i*pow(t, i - 1)*(curveDegree - i)*pow(1 - t, curveDegree - i - 1)
                    + pow(t, i)*(curveDegree - i)*(curveDegree - i - 1)*pow(1 - t, curveDegree - i - 2));
            curveAccelerations[j] = curveAccelerations[j] + controlPoints[i].scale(coefficient);
            t += 1/numPoints;
        }
    }

    // Fix NAN data by extrapolation
    curveAccelerations[0] = curveAccelerations[1] + (curveAccelerations[1] - curveAccelerations[2]);
    curveAccelerations.back() = curveAccelerations[curveAccelerations.size() - 2]
            + (curveAccelerations[curveAccelerations.size() - 2] - curveAccelerations[curveAccelerations.size() - 3]);
}

void CurveCreator::calculateOrientation() {
    for (Vector2 &vel: curveVelocities) {
        curveOrientations.push_back((float) vel.angle());
    }
}

double CurveCreator::factorial(float x) {
    double result = 1;
    for (int i = 1; i <= x; i += 1) {
        result *= i;
    }
    return result;
}

// GETTERS
const std::vector<Vector2> &CurveCreator::getCurvePositions() const {
    return curvePositions;
}

const std::vector<Vector2> &CurveCreator::getCurveVelocities() const {
    return curveVelocities;
}

const std::vector<float> &CurveCreator::getCurveOrientations() const {
    return curveOrientations;
}

double CurveCreator::getTotalTime() const {
    return totalTime;
}
} // ai
} // rtt


