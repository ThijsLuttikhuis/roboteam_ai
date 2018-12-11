//
// Created by mrlukasbos on 23-10-18.
//

#ifndef ROBOTEAM_AI_CONSTANTS_H
#define ROBOTEAM_AI_CONSTANTS_H
//TODO: add units to the below things, check with control/robothub.

#include <QColor>
#include "math.h"

namespace rtt {
namespace ai {
namespace constants {
//
const int tickRate=50;// Rate at which we tick our behavior Trees

//Mathematical constants
const double PI = 3.14159; // TODO: Why do we need this when we have M_PI from math.h? Conflicting usages? Global PI definition is very needed.

//skills
const double DEFAULT_KICK_POWER = 5.0; // max kick power = 100
const int MAX_KICK_CYCLES = 20;
const int MAX_GENEVA_CYCLES = 20;
const int DEFAULT_GENEVA_STATE = 0;

const double FRONT_LENGTH=0.118;
const double ROBOT_RADIUS=0.089; // TODO: Need to test if world_state agrees with this definition of the centre of the robot
const double DRIBBLER_ANGLE_OFFSET=asin(FRONT_LENGTH/2/ROBOT_RADIUS);
const double MAX_BALL_RANGE=0.3; // Could maybe be even less? TODO: needs to be tested.
const double DRIBBLE_POSDIF=0.03;
const float  DRIBBLE_SPEED=0.5;

//Other/multiple usage
const int DEFAULT_ROBOT_ID = 1;
const double MAX_ANGULAR_VELOCITY = 6.0; // rad per second??

//Keeper
const double KEEPER_POST_MARGIN=0.08;//m
const double KEEPER_CENTREGOAL_MARGIN=0.3;//m

// Interface
const int ROBOT_DRAWING_SIZE = 8;
const int BALL_DRAWING_SIZE = 5;
const int TACTIC_COLOR_DRAWING_SIZE = 10;
const int WINDOW_FIELD_MARGIN = 5;

// Settings
const bool STD_SHOW_ROLES = false;
const bool STD_SHOW_TACTICS = false;
const bool STD_SHOW_TACTICS_COLORS = true;
const bool STD_SHOW_VELOCITIES = true;
const bool STD_SHOW_ANGLES = true;
const bool STD_SHOW_VORONOI = false;
const bool STD_SHOW_PATHS_ALL = false;
const bool STD_SHOW_PATHS_CURRENT = false;

const QColor FIELD_COLOR{30, 30, 30, 255};
const QColor FIELD_LINE_COLOR = Qt::white;
const QColor ROBOT_US_COLOR { 150, 150, 255, 255 }; // Blue
const QColor ROBOT_THEM_COLOR { 255, 255, 0, 255 }; // Yellow
const QColor BALL_COLOR { 255, 120, 50, 255 }; // Orange
const QColor TEXT_COLOR = Qt::white;
const QColor SELECTED_ROBOT_COLOR = Qt::magenta;

const QColor TACTIC_1 { 255, 0, 255, 255 };
const QColor TACTIC_2 { 0, 255, 255, 255 };
const QColor TACTIC_3 { 255, 255, 0, 255 };
const QColor TACTIC_4 { 255, 120, 180, 255 };
const QColor TACTIC_5 { 255, 100, 255, 255 };
const QColor TACTIC_COLORS[] = {TACTIC_1, TACTIC_2, TACTIC_3, TACTIC_4, TACTIC_5};

} // constants
} // ai
} // rtt


enum class RefGameState {
    // Ref states as dictated by RoboCup SSL
    HALT = 0,
    STOP = 1,
    NORMAL_START = 2,
    FORCED_START = 3,
    PREPARE_KICKOFF_US = 4,
    PREPARE_KICKOFF_THEM = 5,
    PREPARE_PENALTY_US = 6,
    PREPARE_PENALTY_THEM = 7,
    DIRECT_FREE_US = 8,
    DIRECT_FREE_THEM = 9,
    INDIRECT_FREE_US = 10,
    INDIRECT_FREE_THEM = 11,
    TIMEOUT_US = 12,
    TIMEOUT_THEM = 13,
    GOAL_US = 14,
    GOAL_THEM = 15,
    BALL_PLACEMENT_US = 16,
    BALL_PLACEMENT_THEM = 17,

    // Custom extended refstates
    // These numbers will never be called from the referee immediately, they can only be used as follow-up commands
    DO_KICKOFF = 18,
    DEFEND_KICKOFF = 19,
    DO_PENALTY = 20,
    DEFEND_PENALTY = 21,
    UNDEFINED = -1
};

#endif //ROBOTEAM_AI_CONSTANTS_H
