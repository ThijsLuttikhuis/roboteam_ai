//
// Created by simen on 30/10/18.
//

#include "FindShortestPath.h"

namespace rtt {
    namespace ai {
        rtt::ai::FindShortestPath::FindShortestPath() {

        }

        std::vector<rtt::Vector2> rtt::ai::FindShortestPath::calculateShortestPath(const arma::Mat<float> &voronoiNodes,
                                                                                          const arma::Mat<int> &voronoiSegments,
                                                                                          const int startID, const int endID,
                                                                                          const std::vector<rtt::Vector2> robotCoordinates,
                                                                                          const float startOrientation,
                                                                                          const float endOrientation) {
            nodes = createNodes(voronoiNodes, startID, endID);

            queue.push_back(startID);
            while (queue[0] != endID) {
                expandParentNode(voronoiNodes, voronoiSegments, robotCoordinates, startID, endID, startOrientation, endOrientation);
            }

            return backtrackPath(startID, endID);
        }

        std::vector<FindShortestPath::Node> FindShortestPath::createNodes(const arma::Mat<float> &voronoiNodes, const int startID, const int endID) {
            auto numNodes = voronoiNodes.n_rows;
            std::vector<Node> nodes;

            rtt::Vector2 endPosition;
            for (int i=0; i<numNodes; i++) {
                if (voronoiNodes(i,0) == endID) {
                    endPosition = rtt::Vector2(voronoiNodes(i,1), voronoiNodes(i,2));
                    break;
                }
            }

            for (int i=0; i<numNodes; i++) {
                Node newNode;
                newNode.ID = (int)voronoiNodes(i,0);
                newNode.pos = Vector2(voronoiNodes(i,1), voronoiNodes(i,2));
                newNode.dist_to_end = (float)newNode.pos.dist(endPosition);
                newNode.cost = NAN;

                if (newNode.ID == startID) {
                    newNode.visited = true;
                    newNode.dist_to_start = 0;
                } else {
                    newNode.visited = false;
                    newNode.dist_to_start = std::numeric_limits<float>::max(); // distance to start is infinite by default
                }

                nodes.push_back(newNode);
            }

            return nodes;
        }

        void FindShortestPath::expandParentNode(const arma::Mat<float> &voronoiNodes, const arma::Mat<int> &voronoiSegments,
                                                const std::vector<rtt::Vector2> robotCoordinates, const int startID, const int endID,
                                                const float startOrientation, const float endOrientation) {
            int parentID = queue[0];
            // Erase node from list, it is done now
            queue.erase(queue.begin());
            nodes[findIndex(parentID)].visited = true;
            std::vector<int> childNodeIDs = findChildrenID(parentID, voronoiSegments);

            for (int childID: childNodeIDs) {
                Node child = nodes[findIndex(childID)];
                if (!child.visited) {
                    float dist_to_start = nodes[findIndex(parentID)].dist_to_start + (float)child.pos.dist(nodes[findIndex(parentID)].pos);

                    // Calculate cost
                    float weight = calculateWeight(voronoiSegments, startOrientation, endOrientation, robotCoordinates, startID, endID, childID, parentID);
                    float childCost = (dist_to_start + child.dist_to_end) * weight;
                    if (std::isnan(child.cost) or child.cost > childCost) {
                        nodes[findIndex(childID)].cost = childCost;
                        // update node info
                        nodes[findIndex(childID)].dist_to_start = dist_to_start;
                        nodes[findIndex(childID)].prevNodeID = parentID;
                    }
                    updateQueue(childID); // Put current child in the queue
                }
            }
        }

        void FindShortestPath::updateQueue(int childID) {
            if (queue.empty()) {
                queue.push_back(childID);
            } else {
                for (int i=0; i<queue.size(); i++) {
                    int ID = queue[i];
                    if (nodes[findIndex(childID)].cost < nodes[findIndex(ID)].cost) {
                        queue.insert(queue.begin() + i, childID);
                        break;
                    } else if (i == queue.size()-1) {
                        queue.push_back(childID);
                        break;
                    }
                }
            }
        }

        std::vector<int> FindShortestPath::findChildrenID(int parentID, const arma::Mat<int> &voronoiSegments) {
            // This function would not be necessary when using graphs
            std::vector<int> childNodeIDs;
            for (int i=0; i<voronoiSegments.n_rows; i++) {
                if (voronoiSegments(i,1) == parentID) {
                    childNodeIDs.push_back(voronoiSegments(i,2)); // Add child node to vector
                } else if (voronoiSegments(i,2) == parentID) {
                    childNodeIDs.push_back(voronoiSegments(i,1)); // Add child node to vector
                }
            }
            return childNodeIDs;
        }

        int FindShortestPath::findIndex(int ID) {
            // Finds vector index for node with ID: 'ID'
            int index = NAN;
            for (int i=0; i<nodes.size(); i++) {
                if (nodes[i].ID == ID) {
                    index = i;
                    break;
                }
            }
            return index;
        }

        std::vector<rtt::Vector2> FindShortestPath::backtrackPath(int startID, int endID) {
            std::vector<rtt::Vector2> path;
            path.push_back(nodes[findIndex(endID)].pos);
            int nextID = endID;

            while (nextID != startID) {
                nextID = nodes[findIndex(nextID)].prevNodeID;
                path.insert(path.begin(), nodes[findIndex(nextID)].pos);
            }

            return path;
        }

        float rtt::ai::FindShortestPath::calculateWeight(const arma::Mat<int> &voronoiSegments,
                                                                const float startOrientation, const float endOrientation,
                                                                const std::vector<rtt::Vector2> robotCoordinates, const int startID,
                                                                const int endID, const int childID, const int parentID) {
            // This function could be put in another file, that would decrease the number of inputs in calculateShortestPath
            float orientationFactor = 100; // TODO: Should be optimized
            float robotDiameter = 0.18; // TODO: Should be pulled from some list with constants
            float normalDist = 2*robotDiameter; // normalized distance from object to line. Weight will be 2 at this distance.

            if (parentID == startID) {
                // Segment is connected to start node
                rtt::Vector2 startToChild = nodes[findIndex(childID)].pos - nodes[findIndex(startID)].pos;
                float angleToChild = startOrientation - (float) startToChild.angle();
                return orientationFactor *
                       (float) pow(sin(angleToChild / 2), 2); // mapping function: 0 -> 0, pi -> 1
            }

            if (childID == endID) {
                // Segment is connected to end node
                rtt::Vector2 endToParent = nodes[findIndex(parentID)].pos - nodes[findIndex(endID)].pos;
                float angleToParent = endOrientation - (float) endToParent.angle();
                return orientationFactor *
                       (float) pow(cos(angleToParent / 2), 2); // mapping function: 0 -> 1, pi -> 0
            }

            // Segment is neither connected to the start node nor the end node
            rtt::Vector2 linepoint1 = nodes[findIndex(parentID)].pos;
            rtt::Vector2 linepoint2 = nodes[findIndex(childID)].pos;
            std::vector<float> distances;
            for (rtt::Vector2 objectPos: robotCoordinates) {
                distances.push_back(distancePointToLine(objectPos, linepoint1, linepoint2));
            }
            float minDistance = *std::min_element(distances.begin(), distances.end());
            return 1 + normalDist / minDistance; // goes as 1/x and converges to 1
        }

        float
        FindShortestPath::distancePointToLine(rtt::Vector2 point, rtt::Vector2 linepoint1, rtt::Vector2 linepoint2) {
            // use vector formulation
            // line: x = a + t*n, where a = linepoint1
            // point: p
            rtt::Vector2 n = (linepoint2-linepoint1).normalize();
            float t = n.dot(point-linepoint1);
            if (t < 0) {
                // linepoint1 is closest to point
                return (float)point.dist(linepoint1);
            } else if (t > (linepoint2-linepoint1).length()) {
                // linepoint 2 is closest to point
                return (float)point.dist(linepoint2);
            } else {
                // closest point is on the line segment
                return (float)(linepoint1-point+n.stretchToLength(t)).length();
            }
        }
    }
}

