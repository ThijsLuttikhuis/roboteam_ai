//
// Created by baris on 16/11/18.
//

#ifndef ROBOTEAM_AI_CONTROLUTILS_H
#define ROBOTEAM_AI_CONTROLUTILS_H

namespace control {
class ControlUtils {
    public:
        typedef struct {
          float kP;
          float kI;
          float kD;
          float prev_err;
          float timeDiff;
        } PIDvariables;

        static double calculateAngularVelocity(double robotAngle, double targetAngle);
        static float PIDcontroller(float err, PIDvariables &K);
};
}

#endif //ROBOTEAM_AI_CONTROLUTILS_H
