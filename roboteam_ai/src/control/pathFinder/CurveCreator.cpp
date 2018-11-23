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

void CurveCreator::createCurvePiece(std::vector<Vector2> pathPiece,
        std::vector<Vector2> robotCoordinates) {
    if (pathPiece.size() == 1) {
        std::cout << "Oops, only 1 path node?" << std::endl;
    }
    else if (pathPiece.size() == 2) {
        controlPoints.push_back(pathPiece[1]);
    }
    else {
        controlPoints.push_back(pathPiece[0]
                + (pathPiece[1] - pathPiece[0]).scale(1)); // TODO: .scale() could be used to minimize curvature
        std::vector<Vector2> dangerZone; // Danger zone is the area contained by the next three path nodes
        std::vector<Vector2> mostDangerousObstacle; // Obstacle in dangerZone closest to first path line
        for (int i = 0; i < pathPiece.size() - 2; i ++) {
            dangerZone = {pathPiece[i], pathPiece[i + 1], pathPiece[i + 2]};
            mostDangerousObstacle = findMostDangerousObstacle(dangerZone, robotCoordinates);
            if (mostDangerousObstacle.empty()) {
                controlPoints.push_back(pathPiece[i + 1] + (pathPiece[i + 2] - pathPiece[i + 1]).scale(
                        1)); // TODO: .scale() could be used to minimize curvature
            }
            else {
                // calculate control point
                Vector2 maxOfControlPoint = pointOnLinePastObstacle(dangerZone[0], mostDangerousObstacle[0],
                        {dangerZone[1], dangerZone[2]});
                controlPoints.push_back(pathPiece[i + 1] + (maxOfControlPoint - pathPiece[i + 1]).scale(
                        1)); // TODO: .scale() could be used to minimize curvature
                return;
            }
        }
    }
}

std::vector<Vector2> CurveCreator::findMostDangerousObstacle(std::vector<Vector2> trianglePoints,
        std::vector<Vector2> robotCoordinates) {
    std::vector<Vector2> mostDangerousObstacle;
    for (Vector2 &obstaclePos: robotCoordinates) {
        if (isObstacleInTriangle(trianglePoints, obstaclePos)) {
            if (mostDangerousObstacle.empty()) {
                mostDangerousObstacle.push_back(obstaclePos);
            }
            else {
                Vector2 startToCurrentObstacle = mostDangerousObstacle[0] - trianglePoints[0];
                Vector2 startToNewObstacle = obstaclePos - trianglePoints[0];
                Vector2 pathLine = trianglePoints[1] - trianglePoints[0];

                float anglePathToCurrentObstacle = (float) abs(pathLine.angle() - startToCurrentObstacle.angle())
                        - (float) atan(robotDiameter/startToCurrentObstacle.length());
                float anglePathToNewObstacle = (float) abs(pathLine.angle() - startToNewObstacle.angle())
                        - (float) atan(robotDiameter/startToNewObstacle.length());

                if (anglePathToNewObstacle < anglePathToCurrentObstacle) {
                    mostDangerousObstacle.pop_back();
                    mostDangerousObstacle.push_back(obstaclePos);
                }
            }
        }
    }
    return mostDangerousObstacle;
}

std::vector<Vector2> CurveCreator::createConvexHull(std::vector<Vector2> curvePiece) {
    /// Use Graham Scan to turn curve piece into Convex Hull
    // Find point P that has the lowest y-coordinate
    Vector2 P(DBL_MAX, DBL_MAX);
    for (Vector2 &point : curvePiece) {
        if (point.y < P.y) {
            P = point;
        }
    }

    // Sort vector based on angle between P -> point and x-axis
    std::vector<Vector2> sortedList;
    sortedList.push_back(P);

    Vector2 unitVecX(1, 0);
    for (Vector2 &point : curvePiece) {
        if (point != P) {
            for (int i = 1; i < sortedList.size(); i ++) {
                if ((point - P).normalize().dot(unitVecX) > (sortedList[i] - P).normalize().dot(unitVecX)) {
                    sortedList.insert(sortedList.begin() + i, point);
                    break;
                }
            }
        }
    }

    // Iterate over points and check whether making a left or right turn
    std::vector<Vector2> convex;
    convex.push_back(P);
    convex.push_back(sortedList[1]);

    double crossProduct;
    int index = 0;
    Vector2 vec1, vec2;
    for (Vector2 &point : sortedList) {
        vec1 = convex[index+1] - convex[index];
        vec2 = point - convex[index];
        crossProduct = vec1.x * vec2.y - vec1.y * vec2.x;

        if (crossProduct < 0) {
            // Right turn, erase last point from convex
            convex.pop_back();
        }
        else {
            // Left turn, add point to convex
            convex.push_back(point);
            index++;
        }
    }

    return convex;
}

bool CurveCreator::isObstacleInTriangle(std::vector<Vector2> trianglePoints, Vector2 obstaclePos) {
    if (trianglePoints.size() != 3) {
        std::cout << "Please enter 3 points, not " << trianglePoints.size() << "!" << std::endl;
        return false;
    }

    // Draw lines from the obstacle to the triangle points. If the sum of the angles between these lines is equal to 2*PI, then the obstacle is inside the triangle
    float sumOfAngles = 0;
    sumOfAngles += acos(
            (trianglePoints[0] - obstaclePos).normalize().dot((trianglePoints[1] - obstaclePos).normalize()));
    sumOfAngles += acos(
            (trianglePoints[1] - obstaclePos).normalize().dot((trianglePoints[2] - obstaclePos).normalize()));
    sumOfAngles += acos(
            (trianglePoints[2] - obstaclePos).normalize().dot((trianglePoints[0] - obstaclePos).normalize()));

    if (abs(sumOfAngles - 2*M_PI) < 0.002*M_PI) { // Allow a deviation of 0.1 percent
        return true;
    }

    // If the obstacle is on one of the vertices, it is also inside the triangle.
    bool obstacleOnVertex1 = (distancePointToLine(obstaclePos, trianglePoints[0], trianglePoints[1]) < robotDiameter/2);
    bool obstacleOnVertex2 = (distancePointToLine(obstaclePos, trianglePoints[1], trianglePoints[2]) < robotDiameter/2);
    bool obstacleOnVertex3 = (distancePointToLine(obstaclePos, trianglePoints[2], trianglePoints[0]) < robotDiameter/2);

    return (obstacleOnVertex1 or obstacleOnVertex2 or obstacleOnVertex3);
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

std::vector<Vector2>
CurveCreator::combineCurves(std::vector<Vector2> curve1, std::vector<Vector2> curve2) {
    std::vector<Vector2> totalCurve;

    if (curve1.empty()) {
        totalCurve = curve2;
    }
    else {
        float n1 = curve1.size(); // order of first curve
        float n2 = curve2.size(); // order of second curve
        float a = n1/n2;
        float b = (n1 - 1)/(n2 - 1);

        Vector2 P0 = curve1.back(); // common point
        Vector2 Pmin2 = curve1[curve1.size() - 2];
        Vector2 P2 = curve2[1];

        // Calculate control points to add in order to assure continuity
        Vector2 Pmin1 = P0.scale(1 + 1/(2*a) + b/2)/(1 + b) + Pmin2.scale(b/(2 + 2*b)) + P2.scale(1/(2*a + 2*a*b));
        Vector2 P1 = P0.scale(1 + a) - Pmin1.scale(a);

        totalCurve.insert(totalCurve.begin(), curve1.begin(), curve1.end() - 1);
        totalCurve.push_back(Pmin1);
        totalCurve.push_back(P0);
        totalCurve.push_back(P1);
        totalCurve.insert(totalCurve.end(), curve2.begin() + 1, curve2.end());
    }

    return totalCurve;
}

void CurveCreator::calculateControlPoints(std::vector<Vector2> pathNodes, std::vector<Vector2> &robotCoordinates,
        float startVelocity, float endVelocity) {
    if (pathNodes.size() == 2) {
        controlPoints.push_back(pathNodes[0]);
        controlPoints.push_back(pathNodes[1]);
    }
    else {
        int startNodeIndex = 1;
        int numberOfCurvePieces = 1;
        std::vector<Vector2> pathPiece;
        std::vector<int> curvePieceEdgeIndices;
        curvePieceEdgeIndices.push_back(0);

        controlPoints.push_back(pathNodes[0]); // First path node is always a control point
        while ((controlPoints.back().dist(pathNodes.back())) > 0.01) {
            pathPiece.clear();
            pathPiece.push_back(controlPoints.back());
            for (int i = startNodeIndex; i < pathNodes.size(); i ++) {
                pathPiece.push_back(pathNodes[i]);
            }
            createCurvePiece(pathPiece, robotCoordinates);
            startNodeIndex = (int) controlPoints.size() - numberOfCurvePieces;
            numberOfCurvePieces ++;
            curvePieceEdgeIndices.push_back((int) controlPoints.size());
        }

        // set initial and final velocity
        addVelocityControlPoints(startVelocity, endVelocity, numberOfCurvePieces);

        // combine curves
        if (curvePieceEdgeIndices.size() > 2) {
            std::vector<Vector2> combinedControlPoints;
            for (int j = 0; j < curvePieceEdgeIndices.size() - 1; j ++) {
                std::vector<Vector2> curve2(controlPoints.begin() + curvePieceEdgeIndices[j],
                        controlPoints.begin() + curvePieceEdgeIndices[j + 1] + 1);
                combinedControlPoints = combineCurves(combinedControlPoints, curve2);
            }
            controlPoints = combinedControlPoints;
        }
    }
}

void CurveCreator::addVelocityControlPoints(float startVelocity, float endVelocity, int numberOfCurvePieces) {
    // add two control points to assure the given velocities
//    startVelocity *= totalTime;
//    endVelocity *= totalTime;
    auto numControlPoints = controlPoints.size() + 2*(numberOfCurvePieces
            - 1); // current number of CP's + 2 CP's from this function + all CP's that combineCurves adds

    float startScaleFactor = startVelocity/(numControlPoints - 1);
    startScaleFactor = startScaleFactor > (float) (controlPoints[1] - controlPoints[0]).length()
                       ? (float) (controlPoints[1] - controlPoints[0]).length() : startScaleFactor;
    Vector2 startVelCP = controlPoints[0] + (controlPoints[1] - controlPoints[0]).stretchToLength(startScaleFactor);
    controlPoints.insert(controlPoints.begin() + 1, startVelCP);

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


