//
// Created by selina on 10/30/18.
//

#ifndef ROBOTEAM_TESTSUITE_VORONOICREATOR_H
#define ROBOTEAM_TESTSUITE_VORONOICREATOR_H

#include "roboteam_msgs/World.h"
#include <roboteam_msgs/GeometryData.h>
#include <armadillo>
#include "roboteam_utils/Vector2.h"
#include <cfloat>
#include "../utilities/Field.h"

namespace rtt{
    namespace testsuite{
        class VoronoiCreator{
        private:
            // Functions
            arma::Mat<int> possibleCombinations(arma::Mat<float> objectCoordinates);

            void lineFromPoints(std::pair<float, float> P, std::pair<float, float> Q, double &a, double &b, double &c);

            void perpendicularBisectorFromLine(std::pair<float, float> P, std::pair<float, float> Q, double &a,
                    double &b, double &c);

            std::pair<float, float> lineLineIntersection(double a1, double b1, double c1, double a2, double b2,
                    double c2);

            std::pair<arma::Mat<float>, arma::Mat<float>> findCircumcircles(arma::Mat<int> triangleCombinations,
                    arma::Mat<float> objectCoordinates);

            std::pair<arma::Mat<float>, arma::Mat<int>> delaunayFilter(arma::Mat<float> objectCoordinates,
                    arma::Mat<float> circleCenters, arma::Mat<float> radius, arma::Mat<int> triangleCombinations);

            std::pair<arma::Mat<int>, arma::Mat<int>> findAdjacentCenter(arma::Mat<int> triangleCombinations);

            arma::mat getIndexColumn(int a);

            std::pair<arma::Mat<int>, arma::Mat<int>> startEndSegmentCreator(arma::Mat<int> triangleCombinations,
                    arma::Mat<float> circleCenters, int startID, int endID);

            arma::Mat<float> angleCalculator(int inp, arma::Mat<float> objectCoordidnates, arma::Mat<float> circleCenters,
                    arma::Mat<int> voronoiSegments);

            std::pair<float, float> orientationNodeCreator(int inp, arma::Mat<float> angles, float orientationAngle,
                    arma::Mat<float> circleCenters, arma::Mat<float> objectCoordinates);

            arma::Mat<float> removeIfInDefenceArea(arma::Mat<float> circleCenters);

            arma::Mat<float> removeIfOutOfField(arma::Mat<float> circleCenters);

        public:
            VoronoiCreator();

            // Struct
            struct parameters {
              arma::Mat<float> nodes;
              arma::Mat<int> segments;
            };

            parameters createVoronoi(arma::Mat<float> objectCoordinates,
                    float startOrientationAngle, float endOrientationAngle);


        };
    }
}

#endif //ROBOTEAM_TESTSUITE_VORONOICREATOR_H

