//
// Created by mrlukasbos on 4-12-18.
//

#ifndef ROBOTEAM_AI_DRAWER_H
#define ROBOTEAM_AI_DRAWER_H


#include <roboteam_utils/Vector2.h>
#include <iostream>
#include <armadillo>

namespace rtt {
namespace ai {
namespace interface {

class Drawer {
public:
    explicit Drawer() = default;

    static void setGoToPosLuThPoints(int id, std::vector<Vector2> points);
    static std::vector<Vector2> getGoToPosLuThPoints(int id);

    static void setVoronoiDiagram(arma::Mat<int> voronoiSegments, arma::Mat<float> voronoiNodes);
    static std::pair<arma::Mat<int>, arma::Mat<float>> getVoronoiDiagram(bool plot);

    static void setBezierCurve(std::vector<Vector2> curvePoints);
    static std::vector<Vector2> getBezierCurve(bool plot);

private:
    static std::map<int, std::vector<Vector2>> GoToPosLuThPoints;
    static std::pair<arma::Mat<int>, arma::Mat<float>> voronoiDiagram;
    static std::vector<Vector2> bezierCurve;
};

} // interface
} // ai
} // rtt
#endif //ROBOTEAM_AI_DRAWER_H
