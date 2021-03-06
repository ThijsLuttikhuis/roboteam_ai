//
// Created by mrlukasbos on 19-10-18.
//

#include "Field.h"

namespace rtt {
namespace ai {

roboteam_msgs::GeometryFieldSize Field::field;

const roboteam_msgs::GeometryFieldSize Field::get_field() {
    return Field::field;
}

void Field::set_field(roboteam_msgs::GeometryFieldSize field) {
    Field::field = field;
}

Vector2 Field::get_our_goal_center() {
    return Vector2(field.field_length/- 2, 0);
}

Vector2 Field::get_their_goal_center() {
    return Vector2(field.field_length/2, 0);
}

bool Field::pointIsInDefenceArea(Vector2 point, bool isOurDefenceArea, float margin) {
    auto penaltyLine = isOurDefenceArea ? field.left_penalty_line : field.right_penalty_line;
    double yTopBound;
    double yBottomBound;
    double xBound = penaltyLine.begin.x;
    if (penaltyLine.begin.y < penaltyLine.end.y) {
        yBottomBound = penaltyLine.begin.y;
        yTopBound = penaltyLine.end.y;
    }
    else {
        yBottomBound = penaltyLine.end.y;
        yTopBound = penaltyLine.begin.y;
    }
    bool yIsWithinDefenceArea = point.y<(yTopBound + margin) && point.y>(yBottomBound - margin);
    bool xIsWithinOurDefenceArea = point.x < (xBound + margin);
    bool xIsWithinTheirDefenceArea = point.x > (xBound - margin);

    return yIsWithinDefenceArea && ((isOurDefenceArea && xIsWithinOurDefenceArea)
            || (! isOurDefenceArea && xIsWithinTheirDefenceArea));
}

} // ai
} // rtt