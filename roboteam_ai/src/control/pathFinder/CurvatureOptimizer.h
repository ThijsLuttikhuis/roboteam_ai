//
// Created by simen on 18/12/18.
//

#ifndef ROBOTEAM_AI_CURVATUREOPTIMIZER_H
#define ROBOTEAM_AI_CURVATUREOPTIMIZER_H

#include <roboteam_utils/Vector2.h>
namespace rtt {
namespace ai {
class CurvatureOptimizer {
    private:
        /// Variables
        std::vector<Vector2> pathNodes;
        std::vector<Vector2> controlPoints;
        std::vector<float> parameters;
        std::vector<float> weights;
        int numPoints = 1000;

        /// Functions
        std::vector<float> calculateTimeVector(float t, int derivative);
        std::vector<float> computeChange(float t);
        float factorial(float x);
        float sigmoid(float x);
        float calculateCurvature();
        void calculateWeights();

    public:
        CurvatureOptimizer();
        std::vector<Vector2> optimizeControlPoints();

        /// SETTERS
        void setPathNodes(const std::vector<Vector2> &pathNodes);
        void setControlPoints(const std::vector<Vector2> &controlPoints);
};
}
}

#endif //ROBOTEAM_AI_CURVATUREOPTIMIZER_H
