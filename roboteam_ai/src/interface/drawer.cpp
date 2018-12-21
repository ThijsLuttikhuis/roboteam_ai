//
// Created by mrlukasbos on 4-12-18.
//

#include "drawer.h"

namespace rtt {
namespace ai {
namespace interface {

// declare static variables
//std::map<int, std::vector<Vector2>> Drawer::GoToPosLuThPoints;
std::pair<arma::Mat<int>, arma::Mat<float>> Drawer::voronoiDiagram;
std::vector<Vector2> Drawer::bezierCurve;
std::map<int, std::vector<std::pair<Vector2, QColor>>> Drawer::GoToPosLuThPoints;
std::mutex Drawer::mutex;

// Setters
void Drawer::setGoToPosLuThPoints(int id, std::vector<std::pair<rtt::Vector2, QColor>> points) {
    std::lock_guard<std::mutex> lock(mutex);

    std::pair<int, std::vector<std::pair<rtt::Vector2, QColor>>> pair{id, std::move(points)};

    GoToPosLuThPoints.erase(id);
    GoToPosLuThPoints.insert(pair);
}

void Drawer::setVoronoiDiagram(arma::Mat<int> voronoiSegments, arma::Mat<float> voronoiNodes) {
    voronoiDiagram = std::make_pair(voronoiSegments, voronoiNodes);
}

void Drawer::setBezierCurve(std::vector<Vector2> curvePoints) {
    bezierCurve = curvePoints;
}

// Getters
std::vector<std::pair<Vector2, QColor>> Drawer::getGoToPosLuThPoints(int id) {
    std::lock_guard<std::mutex> lock(mutex);

    if (GoToPosLuThPoints.find(id) != GoToPosLuThPoints.end()) {
        return GoToPosLuThPoints.at(id);
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

} // interface
} // ai
} // rtt