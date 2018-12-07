//
// Created by mrlukasbos on 4-12-18.
//

#include "drawer.h"

namespace rtt {
namespace ai {
namespace interface {

// declare static variables
std::map<int, std::vector<Vector2>> Drawer::GoToPosLuThPoints;
std::pair<arma::Mat<int>, arma::Mat<float>> Drawer::voronoiDiagram;
std::vector<Vector2> Drawer::bezierCurve;

// Setters
void Drawer::setGoToPosLuThPoints(int id, std::vector<rtt::Vector2> points) {
    std::pair<int, std::vector<rtt::Vector2>> pair{id, std::move(points)};

     GoToPosLuThPoints.erase(id);
     Drawer::GoToPosLuThPoints.insert(pair);
}

void Drawer::setVoronoiDiagram(arma::Mat<int> voronoiSegments, arma::Mat<float> voronoiNodes) {
    voronoiDiagram = std::make_pair(voronoiSegments, voronoiNodes);
}

void Drawer::setBezierCurve(std::vector<Vector2> curvePoints) {
    bezierCurve = curvePoints;
}

// Getters
std::vector<Vector2> Drawer::getGoToPosLuThPoints(int id) {

    if (GoToPosLuThPoints.find(id) != GoToPosLuThPoints.end()) {
        return GoToPosLuThPoints[id];
    }
    return {};
}

std::pair<arma::Mat<int>, arma::Mat<float>> Drawer::getVoronoiDiagram(bool plot) {

    if (plot) {
        return voronoiDiagram;
    }
    return {};
}

std::vector<Vector2> Drawer::getBezierCurve(bool plot) {

    if (plot) {
        return bezierCurve;
    }
    return {};
}

}
}
}