//
// Created by simen on 30/10/18.
//

#ifndef ROBOTEAM_TESTSUITE_FINDSHORTESTPATH_H
#define ROBOTEAM_TESTSUITE_FINDSHORTESTPATH_H

#include "roboteam_msgs/World.h"
#include <armadillo>
#include <roboteam_utils/Vector2.h>

namespace rtt {
    namespace testsuite {
        class FindShortestPath {
            private:
                // Types
                struct Node {
                    int ID;
                    rtt::Vector2 pos;
                    int prevNodeID;
                    float dist_to_start;
                    float dist_to_end;
                    bool visited;
                    float cost;
                };

                // Variables
                std::vector<Node> nodes;
                std::vector<int> queue; // Ordered list of node ID's that are accessible

                // Functions
                std::vector<Node> createNodes(const arma::Mat<float> &voronoiNodes, int startID, int endID);
                void expandParentNode(const arma::Mat<float> &voronoiNodes, const arma::Mat<int> &voronoiSegments, std::vector<rtt::Vector2> robotCoordinates, int startID, int endID, float startOrientation, float endOrientation);
                int findIndex(int ID);
                std::vector<int> findChildrenID(int parentID, const arma::Mat<int> &voronoiSegments);
                void updateQueue(int childID);
                std::vector<rtt::Vector2> backtrackPath(int startID, int endID);
                float calculateWeight(const arma::Mat<int> &voronoiSegments,
                                                float startOrientation, float endOrientation, std::vector<rtt::Vector2> robotCoordinates,
                                                int startID, int endID, int childID, int parentID);
                float distancePointToLine(rtt::Vector2 point, rtt::Vector2 linepoint1, rtt::Vector2 linepoint2);

            public:
                FindShortestPath();
                std::vector<rtt::Vector2> calculateShortestPath(const arma::Mat<float> &voronoiNodes, const arma::Mat<int> &voronoiSegments, int startID, int endID, std::vector<rtt::Vector2> robotCoordinates, float startOrientation, float endOrientation);

        };
    } // testsuite
} // rtt

#endif //ROBOTEAM_TESTSUITE_FINDSHORTESTPATH_H
