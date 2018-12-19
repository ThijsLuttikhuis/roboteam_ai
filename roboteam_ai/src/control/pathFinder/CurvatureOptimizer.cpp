//
// Created by simen on 18/12/18.
//

#include "CurvatureOptimizer.h"

namespace rtt {
namespace ai {
CurvatureOptimizer::CurvatureOptimizer() {

}

/// Shift control points to minimize the curvature
// The start, end, start velocity and end velocity control points will not be shifted.
std::vector<Vector2> CurvatureOptimizer::optimizeControlPoints() {
    if (controlPoints.size() > 4) {
        // Set weights default to 0.5
        for (int i = 0; i < controlPoints.size() - 4; i ++) {
            weights.push_back(0.5);
        }

        calculateWeights();
        for (float B : weights) {
            parameters.push_back(sigmoid(B));
        }
        for (int i = 2; i < controlPoints.size()-1; i++) {
            controlPoints[i] = pathNodes[i-1] + (pathNodes[i]-pathNodes[i-1]).scale(parameters[i]);
        }


    }

    return controlPoints;
}



/// Calculate time vector on a certain time according to Bezier's principle
std::vector<float> CurvatureOptimizer::calculateTimeVector(float t, int derivative) {
    std::vector<float> timeVector;
    int curveDegree = (int) controlPoints.size() - 1;

    float controlPointWeight, coefficient;
    for (int i = 0; i < controlPoints.size(); i ++) {
        controlPointWeight = factorial(curveDegree)/(factorial(i)*factorial(curveDegree - i));
        switch (derivative) {
        case 1:
            coefficient = controlPointWeight*(i*(float) pow(t, i - 1)*(float) pow(1 - t, curveDegree - i)
                    - (float) pow(t, i)*(curveDegree - i)*(float) pow(1 - t, curveDegree - i - 1));
        case 2:
            coefficient =
                    controlPointWeight*((float) pow(i, 2)*(float) pow(t, i - 2)*(float) pow(1 - t, curveDegree - i)
                            - 2*i*(float) pow(t, i - 1)*(curveDegree - i)*(float) pow(1 - t, curveDegree - i - 1)
                            + (float) pow(t, i)*(float) pow(curveDegree - i, 2)
                                    *(float) pow(1 - t, curveDegree - i - 2));
        default: coefficient = controlPointWeight*(float) pow(t, i)*(float) pow(1 - t, curveDegree - i);
        }
        timeVector.push_back(coefficient);
    }

    // Fix NAN data
    if (derivative == 2) {
        timeVector[1] = timeVector[2] - (timeVector[3]-timeVector[2]);
        timeVector[timeVector.size()-2] = timeVector[timeVector.size()-3] - (timeVector[timeVector.size()-4]-timeVector[timeVector.size()-3]);
    }
    if (derivative == 1) {
        timeVector[0] = timeVector[1] - (timeVector[2]-timeVector[1]);
        timeVector[timeVector.size()-1] = timeVector[timeVector.size()-2] - (timeVector[timeVector.size()-3]-timeVector[timeVector.size()-2]);
    }

    return timeVector;
}

/// Compute the amount with which the weights should change
std::vector<float> CurvatureOptimizer::computeChange(float t) {
    std::vector<float> timeVector = calculateTimeVector(t, 0);
    std::vector<float> dTimeVector = calculateTimeVector(t, 1);
    std::vector<float> ddTimeVector = calculateTimeVector(t, 2);

    std::vector<Vector2> dKdQ; // Partial derivative: {curvature} / {control point position}
    std::vector<Vector2> dQdA; // Partial derivative: {control point position} / {parameter}
    std::vector<float> dAdB; // Partial derivative: {parameter} / {weight}

    int lastControlPoint = controlPoints.back() == pathNodes.back() ? (int)controlPoints.size() - 2 : (int)controlPoints.size() - 1;
    dKdQ.emplace_back(Vector2(0,0));
    dKdQ.emplace_back(Vector2(0,0));
    dQdA.emplace_back(Vector2(0,0));
    dQdA.emplace_back(Vector2(0,0));
    for (int i = 2; i < lastControlPoint; i++) {
        float dx = (float)controlPoints[i].x * dTimeVector[i];
        float ddx = (float)controlPoints[i].x * ddTimeVector[i];
        float dy = (float)controlPoints[i].y * dTimeVector[i];
        float ddy = (float)controlPoints[i].y * ddTimeVector[i];

        float firstTerm = (ddTimeVector[i] - dTimeVector[i])/sqrt(dx*dx + dy*dy);
        float secondTerm = -(ddx*dy - dx*ddy)*dTimeVector[i]/(float)pow(dx*dx+dy*dy, 3/2);

        dKdQ.emplace_back(Vector2(firstTerm+secondTerm*dx, firstTerm+secondTerm*dy));
        dQdA.emplace_back(pathNodes[i]-pathNodes[i-1]);
    }
    dKdQ.emplace_back(Vector2(0,0));
    dQdA.emplace_back(Vector2(0,0));
    if (controlPoints.back() == pathNodes.back()) {
        dKdQ.emplace_back(Vector2(0,0));
        dQdA.emplace_back(Vector2(0,0));
    }

    for (float B : weights) {
        dAdB.push_back(sigmoid(B)*(1-sigmoid(B)));
    }

    // Combine all partial derivatives to compute the total change
    std::vector<float> totalChange;
    for (int i = 0; i < controlPoints.size(); i++) {
        totalChange.push_back(((float)dKdQ[i].x*(float)dQdA[i].x + (float)dKdQ[i].y*(float)dQdA[i].y)*dAdB[i]);
    }

    return totalChange;
}

/// Factorial function: result = x!
float CurvatureOptimizer::factorial(float x) {
    float result = 1;
    for (int i = 1; i <= x; i += 1) {
        result *= i;
    }
    return result;
}

/// Sigmoid function: squishes x to a value between 0 and 1
float CurvatureOptimizer::sigmoid(float x) {
    return 1/(1 + exp(-x));
}

/// SETTERS
void CurvatureOptimizer::setPathNodes(const std::vector<Vector2> &pathNodes) {
    CurvatureOptimizer::pathNodes = pathNodes;
}
void CurvatureOptimizer::setControlPoints(const std::vector<Vector2> &controlPoints) {
    CurvatureOptimizer::controlPoints = controlPoints;
}
float CurvatureOptimizer::calculateCurvature() {

    return 0;
}
void CurvatureOptimizer::calculateWeights() {
    float t = 0;
    std::vector<float> change;
    float learningRate = 1; // TODO: magical number, could be just equal to 1?
    for (int i = 0; i < numPoints; i++) {
        change = computeChange(t);
        for (int j = 0; j < weights.size(); j++) {
            weights[i] += learningRate * -change[j];
        }
        t += 1.0/numPoints;
    }
}
} // ai
} // rtt

