//
// Created by selina on 10/30/18.
//

#include "VoronoiCreator.h"

namespace rtt {
namespace ai {

VoronoiCreator::VoronoiCreator() {
}

// Create Voronoi diagram
VoronoiCreator::parameters VoronoiCreator::createVoronoi(const arma::Mat<float> objectCoordinates,
        const float startOrientationAngle,
        const float endOrientationAngle) {

    // Set ID of the start and end point
    const int startID = 0; // always
    const int endID = 1;

    // Calculate all possible triangle combinations
    arma::Mat<int> triangleCombinations = possibleCombinations(objectCoordinates);

    // Calculate the radius and center of each triangle
    std::pair<arma::Mat < float>, arma::Mat < float >> circleParameters = findCircumcircles(triangleCombinations,
            objectCoordinates);

    // Make triangles Delaunay
    std::pair<arma::Mat < float>, arma::Mat < int >> delaunayTriangles = delaunayFilter(objectCoordinates,
            circleParameters.first, circleParameters.second, triangleCombinations);
    arma::Mat<float> circleCenters = delaunayTriangles.first;
    triangleCombinations = delaunayTriangles.second;

    // Add start & end point to circleCenters
    arma::Mat<float> startPoint(1, 2);
    startPoint << objectCoordinates(startID, 0) << objectCoordinates(startID, 1)
               << arma::endr;

    circleCenters.insert_rows(0, startPoint);

    arma::Mat<float> endPoint(1, 2);
    endPoint << objectCoordinates(endID, 0) << objectCoordinates(endID, 1)
             << arma::endr;

    circleCenters.insert_rows(1, endPoint);

    // Add column with 1 to amount of centers to circleCenter for indexing purposes
    // Convert index column to float
    arma::mat temp = getIndexColumn(circleCenters.n_rows);
    arma::Mat<float> indexCenters = arma::conv_to < arma::Mat < float >> ::from(temp);
    circleCenters.insert_cols(0, indexCenters);

    // Find triangles that share a side
    // First = triangles, second = centers
    std::pair<arma::Mat < int>, arma::Mat < int >> adjacent = findAdjacentCenter(triangleCombinations);

    // Create lines from the start & end point to the centers within that polygon
    // First = start, second = end
    std::pair<arma::Mat < int>, arma::Mat < int >> startEndSegments = startEndSegmentCreator(triangleCombinations,
            circleCenters, startID, endID);

    // Combine adjacentCenters and start & end segments to voronoiSegments
    arma::Mat<int> voronoiSegments = adjacent.second;
    voronoiSegments.insert_rows(voronoiSegments.n_rows, startEndSegments.first);
    voronoiSegments.insert_rows(voronoiSegments.n_rows, startEndSegments.second);

    // Add index column to voronoiSegments
    temp = getIndexColumn(voronoiSegments.n_rows);
    arma::Mat<int> indexSegments = arma::conv_to < arma::Mat < int >> ::from(temp);
    voronoiSegments.insert_cols(0, indexSegments);

    // Remove nodes that are in the defence area or outside of the field
    circleCenters = removeIfInDefenceArea(circleCenters, startID, endID);
    circleCenters = removeIfOutOfField(circleCenters, startID, endID);

    // Calculate angle between start & end point and their centers on the surrounding polygon
    arma::Mat<float> anglesStart = angleCalculator(startID, objectCoordinates, circleCenters, startEndSegments.first);
    arma::Mat<float> anglesEnd = angleCalculator(endID, objectCoordinates, circleCenters, startEndSegments.second);

    // Remove start/end segments from segment list
    int amountOfRows = startEndSegments.first.n_rows + startEndSegments.second.n_rows;
    voronoiSegments.shed_rows(voronoiSegments.n_rows - amountOfRows, voronoiSegments.n_rows - 1);

    // Calculate orientation nodes
    std::pair<std::pair<float, float>, std::pair<int, int>> startOrientationParameters =
            orientationNodeCreator(startID, anglesStart, startOrientationAngle, circleCenters, objectCoordinates);
    std::pair<std::pair<float, float>, std::pair<int, int>> endOrientationParameters =
            orientationNodeCreator(endID, anglesEnd, endOrientationAngle, circleCenters, objectCoordinates);

    std::cout << startOrientationParameters.first.first << " " << startOrientationParameters.first.second << std::endl;
    std::cout << endOrientationParameters.first.first << " " << endOrientationParameters.first.second << std::endl;

    std::pair<float, float> startOrientationNode = startOrientationParameters.first;
    std::pair<float, float> endOrientationNode = endOrientationParameters.first;
    std::pair<int, int> startOrientationSegments = startOrientationParameters.second;
    std::pair<int, int> endOrientationSegments = endOrientationParameters.second;

    // Check if segments are valid
    // If not, find node closest to the orientation node and make that a segment
    int s1 = 0;
    int s2 = 0;

    for (int i = 0; i < circleCenters.n_rows; i ++) {
        if ((int) circleCenters(i, 0) == startOrientationSegments.first ||
                circleCenters(i, 0) == startOrientationSegments.second) {
            s1 = 1;
        }
        if ((int) circleCenters(i, 0) == endOrientationSegments.first ||
                circleCenters(i, 0) == endOrientationSegments.second) {
            s2 = 1;
        }
    }

    if (s1 == 0) {
        startOrientationSegments.first = findClosestPoint(startOrientationNode, circleCenters);
        startOrientationSegments.second = startOrientationSegments.first;
    }
    if (s2 == 0) {
        endOrientationSegments.first = findClosestPoint(endOrientationNode, circleCenters);
        endOrientationSegments.second = endOrientationSegments.first;
    }

    // Add start orientation nodes to circleCenters and orientation - start combination to voronoi segments
    // These must never be removed so it needs to happen after the removal of nodes within defence area / outside
    // of the field
    arma::Mat<float> tempRow;
    arma::Mat<int> tempRow1;
    tempRow1 << voronoiSegments(voronoiSegments.n_rows - 1, 0) + 1 << startID
             << circleCenters(circleCenters.n_rows - 1, 0) + 1 << arma::endr;
    tempRow << circleCenters(circleCenters.n_rows - 1, 0) + 1 << startOrientationNode.first
            << startOrientationNode.second << arma::endr;
    voronoiSegments.insert_rows(voronoiSegments.n_rows, tempRow1);
    circleCenters.insert_rows(circleCenters.n_rows, tempRow);

    // Add start orientation segments
    tempRow1.reset();
    tempRow1 << voronoiSegments(voronoiSegments.n_rows - 1, 0) + 1 << circleCenters(circleCenters.n_rows - 1, 0)
             << startOrientationSegments.first << arma::endr;
    voronoiSegments.insert_rows(voronoiSegments.n_rows, tempRow1);

    tempRow1 << voronoiSegments(voronoiSegments.n_rows - 1, 0) + 1 << circleCenters(circleCenters.n_rows - 1, 0)
             << startOrientationSegments.second << arma::endr;
    voronoiSegments.insert_rows(voronoiSegments.n_rows, tempRow1);

    // Add end orientation nodes to circleCenters and orientation - end combination to voronoi segments
    tempRow.reset();
    tempRow1.reset();
    tempRow1 << voronoiSegments(voronoiSegments.n_rows - 1, 0) + 1 << endID
             << circleCenters(circleCenters.n_rows - 1, 0) + 1 << arma::endr;
    tempRow << circleCenters(circleCenters.n_rows - 1, 0) + 1 << endOrientationNode.first
            << endOrientationNode.second << arma::endr;
    voronoiSegments.insert_rows(voronoiSegments.n_rows, tempRow1);
    circleCenters.insert_rows(circleCenters.n_rows, tempRow);

    // Add end orientation segments
    tempRow1.reset();
    tempRow1 << voronoiSegments(voronoiSegments.n_rows - 1, 0) + 1 << circleCenters(circleCenters.n_rows - 1, 0)
             << endOrientationSegments.first << arma::endr;
    voronoiSegments.insert_rows(voronoiSegments.n_rows, tempRow1);
    tempRow1.reset();
    tempRow1 << voronoiSegments(voronoiSegments.n_rows - 1, 0) + 1 << circleCenters(circleCenters.n_rows - 1, 0)
             << endOrientationSegments.second << arma::endr;
    voronoiSegments.insert_rows(voronoiSegments.n_rows, tempRow1);

    // Change name for the rest of the calculations
    parameters voronoiParameters;
    voronoiParameters.nodes = circleCenters;
    voronoiParameters.segments = voronoiSegments;

    return voronoiParameters;
}

// Create all possible triangle combinations with the given amount of objects
arma::Mat<int> VoronoiCreator::possibleCombinations(const arma::Mat<float> objectCoordinates) {
    // Variables
    auto nObjects = objectCoordinates.n_rows;
    arma::Mat<int> triangleCombinations;

    int p = 0;
    for (int i = 0; i < (nObjects - 2); i ++) {
        for (int l = i + 1; l < (nObjects - 1); l ++) {
            for (int k = l + 1; k < nObjects; k ++) {
                arma::Mat<int> combination;
                combination << i << l << k << arma::endr;
                triangleCombinations.insert_rows(p, combination);
                p ++;
            }
        }
    }

    return triangleCombinations;
}

// Find the line coefficients given two points
void
VoronoiCreator::lineFromPoints(std::pair<float, float> P, std::pair<float, float> Q, double &a,
        double &b, double &c) {
    a = Q.second - P.second;
    b = P.first - Q.first;
    c = a*(P.first) + b*(P.second);
}

// Convert the input line to its perpendicular bisector
// Inputs the points whose mid-point lies on the bisector
void
VoronoiCreator::perpendicularBisectorFromLine(std::pair<float, float> P, std::pair<float, float> Q,
        double &a, double &b, double &c) {
    auto midPoint = std::make_pair((P.first + Q.first)/2, (P.second + Q.second)/2);

    // c = -bx + ay
    c = - b*(midPoint.first) + a*(midPoint.second);

    double temp = a;
    a = - b;
    b = temp;
}

// Determine the intersection point of two lines
std::pair<float, float>
VoronoiCreator::lineLineIntersection(double a1, double b1, double c1, double a2, double b2, double c2) {
    double determinant = a1*b2 - a2*b1;
    if (determinant == 0) {
        // The lines are parallel; there is no intersection
        // Simplify by returning a pair of INT_MAX (so it will be excluded later)
        return std::make_pair(INT_MAX, INT_MAX);
    }
    else {
        double x = (b2*c1 - b1*c2)/determinant;
        double y = (a1*c2 - a2*c1)/determinant;
        return std::make_pair(x, y);
    }
}

// Find the circumcenter and radius of all triangles
std::pair<arma::Mat<float>, arma::Mat<float>>
VoronoiCreator::findCircumcircles(arma::Mat<int> triangleCombinations, arma::Mat<float> objectCoordinates) {
    // Make matrices
    arma::Mat<float> triangleCorners = arma::ones<arma::Mat<float>>(3, 2);
    arma::Mat<float> circleCenters = arma::ones<arma::Mat<float>>(triangleCombinations.n_rows, 2);
    arma::Mat<float> radius = arma::ones<arma::Mat<float>>(triangleCombinations.n_rows, 1);

    // Create matrix with coordinates of the three triangle corners
    for (int i = 0; i < triangleCombinations.n_rows; i ++) {
        for (int k = 0; k < 3; k ++) {
            int index = triangleCombinations(i, k);
            triangleCorners(k, 0) = objectCoordinates(index, 0);
            triangleCorners(k, 1) = objectCoordinates(index, 1);
        }

        // Make coordinates pairs, so they can go in the functions
        std::pair<float, float> A = std::make_pair(triangleCorners(0, 0), triangleCorners(0, 1));
        std::pair<float, float> B = std::make_pair(triangleCorners(1, 0), triangleCorners(1, 1));
        std::pair<float, float> C = std::make_pair(triangleCorners(2, 0), triangleCorners(2, 1));

        // Create line AB
        double a, b, c;
        lineFromPoints(A, B, a, b, c);

        // Create line BC
        double e, f, g;
        lineFromPoints(B, C, e, f, g);

        // Convert lines AB & BC to perpendicular bisectors. After this, ax + by = c, ex + fy = g
        perpendicularBisectorFromLine(A, B, a, b, c);
        perpendicularBisectorFromLine(B, C, e, f, g);

        // Calculate circumcenter
        std::pair<float, float> circumcenter = lineLineIntersection(a, b, c, e, f, g);
        circleCenters(i, 0) = circumcenter.first;
        circleCenters(i, 1) = circumcenter.second;

        // Calculate radius
        float p = circleCenters(i, 0) - triangleCorners(0, 0);
        float q = circleCenters(i, 1) - triangleCorners(0, 1);
        radius(i) = (float) sqrt(pow(p, 2) + pow(q, 2));

    }

    return std::make_pair(circleCenters, radius);
}

// Remove the triangle combination and corresponding triangle center if the circle around the triangle contains
// objects that are in objectCoordinates -> make the triangles Delaunay
std::pair<arma::Mat<float>, arma::Mat<int>>
VoronoiCreator::delaunayFilter(const arma::Mat<float> objectCoordinates, arma::Mat<float> circleCenters,
        arma::Mat<float> radius, arma::Mat<int> triangleCombinations) {

    arma::uvec rowsToRemove(triangleCombinations.n_rows);
    rowsToRemove.fill(0);

    for (int i = 0; i < triangleCombinations.n_rows; i ++) {

        double ccX = circleCenters(i, 0); // circlecenter = cc
        double ccY = circleCenters(i, 1);

        // TODO: use 'don't use corners of triangle to remove rows'
        for (int k = 0; k < objectCoordinates.n_rows; k ++) {
            double ocX = objectCoordinates(k, 0); // objectcoordinate = oc
            double ocY = objectCoordinates(k, 1);
            double distance = sqrt(pow(ccX - ocX, 2) + pow(ccY - ocY, 2));
            float threshold = 0.0001;

            if ((distance < radius(i)) && (abs(distance - radius(i)) > threshold)) {
                rowsToRemove(i) = 1;
            }
        }
    }

    // Remove rows that contain non-delaunay triangles and centers
    circleCenters = circleCenters.rows(find(rowsToRemove == 0));
    triangleCombinations = triangleCombinations.rows(find(rowsToRemove == 0));

    return std::make_pair(circleCenters, triangleCombinations);
}

// Make matrix which contains the triangles that share a side
// For example: triangles 1-2-3 (read: point 1, point 2, point 3) and 234 share side 23 so they are neighbours
// If triangle 123 is triangle no. 1 and triangle 234 is triangle no. 2 then they will be in the same row: [1 2]
// A row [1 2 3] means that triangle no. 1 is connected to triangle no. 2 and triangle no. 3.
std::pair<arma::Mat<int>, arma::Mat<int>>
VoronoiCreator::findAdjacentCenter(arma::Mat<int> triangleCombinations) {
    arma::Mat<int> newCombinations = arma::ones<arma::Mat<int>>(triangleCombinations.n_rows, 4);
    arma::Mat<int> adjacentTriangles(triangleCombinations.n_rows, 10);
    arma::Mat<int> adjacentCenters;
    adjacentTriangles.fill(INT_MAX);
    std::pair<int, int> comb;

    // Empty column to add if it becomes too big
    arma::mat temp = getIndexColumn(triangleCombinations.n_rows);
    arma::Mat<int> emptyColumn = arma::conv_to<arma::Mat<int>>::from(temp);
    emptyColumn.fill(INT_MAX);

    newCombinations(arma::span(0, triangleCombinations.n_rows - 1), arma::span(0, 2)) =
            triangleCombinations(arma::span(0, triangleCombinations.n_rows - 1), arma::span(0, 2));
    newCombinations(arma::span(0, triangleCombinations.n_rows - 1), 3) =
            triangleCombinations(arma::span(0, triangleCombinations.n_rows - 1), 0);

    for (int i = 0; i < triangleCombinations.n_rows; i ++) {
        adjacentTriangles(i, 0) = i;
    }

    int p, q;
    // Don't touch
    for (int i = 0; i < triangleCombinations.n_rows; i ++) {
        p = 1;
        q = (int) adjacentTriangles.n_cols; // value to start inserting a new column

        for (int k = 0; k < 3; k ++) {
            comb = std::make_pair(newCombinations(i, k), newCombinations(i, k + 1));

            for (int l = 0; l < triangleCombinations.n_rows; l ++) {
                for (int m = 0; m < 3; m ++) {
                    if (i != l) {
                        if (newCombinations(l, m) == comb.first && newCombinations(l, m + 1) == comb.second) {
                            adjacentTriangles(i, p) = l;
                            p ++;
                        }
                        if (newCombinations(l, m) == comb.second && newCombinations(l, m + 1) == comb.first) {
                            adjacentTriangles(i, p) = l;
                            p ++;
                        }
                        if (p >= adjacentTriangles.n_cols) {
                            adjacentTriangles.insert_cols(q, emptyColumn);
                            q ++;
                        }
                    }
                }
            }
        }
    }

    p = 0;
    for (int i = 0; i < adjacentTriangles.n_rows; i ++) {
        int firstTriangle = adjacentTriangles(i, 0) + 2;

        for (int k = 1; k < adjacentTriangles.n_cols; k ++) {
            int nextTriangle = adjacentTriangles(i, k) + 2;
            if ((nextTriangle != INT_MAX) && (nextTriangle > firstTriangle)) {
                arma::Mat<int> tempRow(1, 2); // Make row with first and next triangle so they can be inserted
                tempRow(0, 0) = firstTriangle;
                tempRow(0, 1) = nextTriangle;
                adjacentCenters.insert_rows(p, tempRow);
                p ++;
            }
        }
    }

    std::pair<arma::Mat<int>, arma::Mat<int>> adjacent = std::make_pair(adjacentTriangles, adjacentCenters);

    return adjacent;
}

// Make column 0:a
arma::mat VoronoiCreator::getIndexColumn(int a) {
    arma::mat indexColumn(a, 1);
    for (int i = 0; i < a; i ++) {
        indexColumn(i, 0) = i;
    }
    return indexColumn;
}

// Create the segments from the start and end points to their surrounding points on the polygon
std::pair<arma::Mat<int>, arma::Mat<int>>
VoronoiCreator::startEndSegmentCreator(arma::Mat<int> triangleCombinations, arma::Mat<float> circleCenters,
        int startID, int endID) {
    arma::Mat<int> startComb;
    arma::Mat<int> endComb;
    arma::Mat<int> tempComb(1, 2);

    int p = 0;
    int q = 0;

    for (int i = 0; i < triangleCombinations.n_rows; i ++) {
        if (triangleCombinations(i, 0) == startID) {
            if (triangleCombinations(i, 1) == endID) {
                tempComb(0, 0) = endID;
                tempComb(0, 1) = (int) circleCenters(i, 0) + 2;
                endComb.insert_rows(p, tempComb);
                p ++;
                tempComb(0, 0) = startID;
                tempComb(0, 1) = (int) circleCenters(i, 0) + 2;
                startComb.insert_rows(q, tempComb);
                q ++;
            }
            else {
                tempComb(0, 0) = startID;
                tempComb(0, 1) = (int) circleCenters(i, 0) + 2;
                startComb.insert_rows(q, tempComb);
                q ++;
            }
        }
        else if (triangleCombinations(i, 0) == endID) {
            tempComb(0, 0) = endID;
            tempComb(0, 1) = (int) circleCenters(i, 0) + 2;
            endComb.insert_rows(p, tempComb);
            p ++;
        }
    }

    std::pair<arma::Mat<int>, arma::Mat<int>> startEndSegments = std::make_pair(startComb, endComb);

    return startEndSegments;
}

// Calculate angles between the start/end point and the points connected to it
arma::Mat<float>
VoronoiCreator::angleCalculator(const int inp, const arma::Mat<float> objectCoordinates, arma::Mat<float> circleCenters,
        arma::Mat<int> segments) {

    arma::Mat<float> angles;
    arma::Mat<float> temp(1, 2);
    float ptX = objectCoordinates(inp, 0); // x coordinate of the input coordinate (start or end point)
    float ptY = objectCoordinates(inp, 1);

    std::vector<Vector2> polygonCoordinates;
    std::vector<int> indexVec;
    int p = 0;
    int index;

    for (int i = 0; i < segments.n_rows; i ++) {
        index = segments(i, 1);
        for (int k = 0; k < circleCenters.n_rows; k ++) {
            if ((int) circleCenters(k, 0) == index) {
                polygonCoordinates.emplace_back(Vector2(circleCenters(k, 1), circleCenters(k, 2)));
                indexVec.emplace_back(index);
            }
        }
    }

    for (int i = 0; i < polygonCoordinates.size(); i ++) {
        float angle = (float) atan((polygonCoordinates[i].y - ptY)/(polygonCoordinates[i].x - ptX));
        temp << angle << indexVec[i] << arma::endr;

        angles.insert_rows(p, temp);

        // In some cases; pi or 2pi must be added
        polygonCoordinates[i].x <= ptX ? angles(p, 0) = angles(p, 0) + (float) M_PI : angles(p, 0) = angles(p, 0);
        angles(p, 0) < 0 ? angles(p, 0) = angles(p, 0) + 2*(float) M_PI : angles(p, 0) = angles(p, 0);
        p ++;
    }

    return angles;
}

// Calculate point at which position the orientation vector crosses the polygon around the point
std::pair<std::pair<float, float>, std::pair<int, int>>
VoronoiCreator::orientationNodeCreator(const int inp, arma::Mat<float> angles, float orientationAngle,
        arma::Mat<float> circleCenters, const arma::Mat<float> objectCoordinates) {

    std::pair<float, float> orientationNode;
    std::pair<int, int> orientationSegments;

    if (angles.is_empty() && inp == 0) {
        std::cout << "No segments connected to the start point! Can't create orientation node!" << std::endl;
    }
    else if (angles.is_empty() && inp == 1) {
        std::cout << "No segments connected to the end point! Can't create orientation node!" << std::endl;
    }

    std::pair<float, float> pt = std::make_pair(objectCoordinates(inp, 0), objectCoordinates(inp, 1));

    arma::Mat<float> temp(1, 2);
    arma::Mat<float> greaterAngle;
    arma::Mat<float> smallerAngle;

    // If the input is the end point, add pi to end point, because the orientation point must inverted
    inp == 1 ? orientationAngle = orientationAngle - (float) M_PI : orientationAngle = orientationAngle;
    orientationAngle < 0 ? orientationAngle = orientationAngle + (float) 2*M_PI : orientationAngle = orientationAngle;

    int p = 0;
    int q = 0;

    for (int i = 0; i < angles.n_rows; i ++) {
        temp << angles(i, 1) << angles(i, 0) << arma::endr;
        if (angles(i, 0) >= orientationAngle) {
            greaterAngle.insert_rows(p, temp);
            p ++;
        }
        else {
            smallerAngle.insert_rows(q, temp);
            q ++;
        }
    }

    // If the greater or smaller angle does not exist, take the smallest and largest angle respectively in the vector
    // of angles
    if (greaterAngle.is_empty()) {
        float value = angles.col(0).min();
        int index = angles.col(0).index_min();
        temp << angles(index, 1) << value << arma::endr;
        greaterAngle.insert_rows(0, temp);
    }

    if (smallerAngle.is_empty()) {
        float value = angles.col(0).max();
        int index = angles.col(0).index_max();
        temp << angles(index, 1) << value << arma::endr;
        smallerAngle.insert_rows(0, temp);
    }

    // Calculate difference between greater angles and the orientation angle
    arma::Mat<float> angleDifGreater = abs(greaterAngle.col(1) - orientationAngle);
    arma::Mat<float> angleDifSmaller = abs(smallerAngle.col(1) - orientationAngle);

    // The smallest distance is the index point
    // The two index points are the points that the vector is pointing in between
    int indexGreater = angleDifGreater.index_min();
    int indexSmaller = angleDifSmaller.index_min();

    if (smallerAngle(indexSmaller, 0) == greaterAngle(indexGreater, 0)) {
        std::cout << "Can't draw line between one point! Orientation point is now this point." << std::endl;
        for (int i = 0; i < circleCenters.n_rows; i ++) {
            if (i == smallerAngle(indexSmaller, 0)) {
                orientationNode = std::make_pair(circleCenters(i, 1), circleCenters(i, 2));
                orientationSegments = std::make_pair(smallerAngle(indexSmaller, 0), inp);
            }
        }
    }
    else {
        arma::Mat<float> adjacentAngle;
        adjacentAngle.insert_rows(0, greaterAngle.row(indexGreater));
        adjacentAngle.insert_rows(1, smallerAngle.row(indexSmaller));

        // Determine the coordinates of the points that the orientation vector is pointing in between
        float xg, yg, xs, ys; // x greater, y greater, x smaller, y smaller
        for (int i = 0; i < circleCenters.n_rows; i ++) {
            if (circleCenters(i, 0) == adjacentAngle(0, 0)) {
                xg = circleCenters(i, 1);
                yg = circleCenters(i, 2);
            }
            else if (circleCenters(i, 0) == adjacentAngle(1, 0)) {
                xs = circleCenters(i, 1);
                ys = circleCenters(i, 2);
            }
        }

        std::pair<float, float> linePoint1 = std::make_pair(xg, yg);
        std::pair<float, float> linePoint2 = std::make_pair(xs, ys);

        // Create a point at some distance in front of the start/end point to be able to create a line between
        // this point and the start/end point
        float orientationMargin = 0.01; // random value, can be anything
        float h = orientationMargin*sin(orientationAngle);
        float l = orientationMargin*cos(orientationAngle);
        std::pair<float, float> ptOrientation = std::make_pair(pt.first + l, pt.second + h);

        // Create lines and calculate intersection
        // line 1 = start - ptOrientation
        // line 2 = linePoints1 - linePoints2

        // Create line 1
        double a, b, c;
        lineFromPoints(pt, ptOrientation, a, b, c);

        // Create line 2
        double e, f, g;
        lineFromPoints(linePoint1, linePoint2, e, f, g);

        // Calculate orientation node
        orientationNode = lineLineIntersection(a, b, c, e, f, g);

        // Put orientation point on field line if it's outside of the field
        float length = Field::get_field().field_length;
        float width = Field::get_field().field_width;
        if (orientationNode.first > length/2 || orientationNode.first < - length/2) {
            orientationNode.first < 0 ? orientationNode.first = - length/2 : orientationNode.first = length/2;
        }
        if (orientationNode.second > width/2 || orientationNode.second < - width/2) {
            orientationNode.second < 0 ? orientationNode.second = - width/2 : orientationNode.second = width/2;
        }

        // Make pair of points that should be connected to the orientation node
        orientationSegments = std::make_pair(adjacentAngle(0, 0), adjacentAngle(1, 0));
    }

    std::pair<std::pair<float, float>, std::pair<int, int>> orientationParameters = std::make_pair(orientationNode,
            orientationSegments);

    return orientationParameters;
}

arma::Mat<float> VoronoiCreator::removeIfInDefenceArea(arma::Mat<float> circleCenters, int startID, int endID) {

    Vector2 point;
    arma::uvec rowsToRemove(circleCenters.n_rows);
    rowsToRemove.fill(0);

    for (int i = 0; i < circleCenters.n_rows; i ++) {
        if (circleCenters(i, 0) != startID && circleCenters(i, 0) != endID) {
            point = Vector2(circleCenters(i, 1), circleCenters(i, 2));
            if (Field::pointIsInDefenceArea(point, true, 0) || Field::pointIsInDefenceArea(point, false, 0)) {
                rowsToRemove(i) = 1;
            }
        }
    }

    // Remove center if the center is within the defence area
    circleCenters = circleCenters.rows(find(rowsToRemove == 0));

    return circleCenters;
}

arma::Mat<float> VoronoiCreator::removeIfOutOfField(arma::Mat<float> circleCenters, int startID, int endID) {
    float width = Field::get_field().field_width; // returns 0
    float length = Field::get_field().field_length; // returns 0

    arma::uvec rowsToRemove(circleCenters.n_rows);
    rowsToRemove.fill(0);

    arma::Mat<float> fieldEdges;
    fieldEdges << length/2 << width/2 << arma::endr
               << - length/2 << width/2 << arma::endr
               << - length/2 << - width/2 << arma::endr
               << length/2 << - width/2 << arma::endr;

    for (int i = 0; i < circleCenters.n_rows; i ++) {
        if (circleCenters(i, 0) != startID && circleCenters(i, 0) != endID) {
            if ((circleCenters(i, 1) > fieldEdges(0, 0)) || (circleCenters(i, 1) < fieldEdges(1, 0)) ||
                    (circleCenters(i, 2) > fieldEdges(0, 1) || (circleCenters(i, 2) < fieldEdges(2, 1)))) {
                rowsToRemove(i) = 1;
            }
        }
    }

    // Remove centers if they are outside of the field
    circleCenters = circleCenters.rows(find(rowsToRemove == 0));

    return circleCenters;
}
int VoronoiCreator::findClosestPoint(std::pair<float, float> node, arma::Mat<float> circleCenters) {

    arma::Mat<float> distance(circleCenters.n_rows - 2, 1); // center number, distance
    float x, p, q;

    for (int i = 2; i < circleCenters.n_rows; i ++) {
        p = node.first - circleCenters(i, 1);
        q = node.second - circleCenters(i, 2);
        x = (float) sqrt(pow(p, 2) + pow(q, 2));
        distance(i - 2, 0) = x;
    }

    int closestPoint = distance.index_min();

    return closestPoint;
}

} // ai
} // rtt