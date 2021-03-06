//
// Created by kjhertenberg on 18-12-18.
//

#ifndef ROBOTEAM_AI_CONTROLLER_H
#define ROBOTEAM_AI_CONTROLLER_H

#include "ControlUtils.h"

namespace control {

    class Controller {
    private:
        double kP;
        double kI;
        double kD;
        double timeDiff;
        double initial_I;
        double initial_I2; //only used in the case of 2 input variables
        double prev_error;
        double prev_error2;

    public:
        Controller();
        Controller(double P, double I, double D);
        Controller(double P, double I, double D, double timeDiff);
        Controller(double P, double I, double D, double timeDiff, double initial, double initial2, double prev, double prev2);

        void setP(double P);
        void setI(double I);
        void setD(double D);
        void setInitial(double initial);
        void setTimeDiff(double time);
        void setPrevErr(double prev);

        void setP(double P, double time);
        void setI(double I, double time);
        void setD(double D, double time);
        void setPI(double P, double I);
        void setPI(double P, double I, double time);
        void setPD(double P, double D);
        void setPD(double P, double D, double time);
        void setID(double I, double D);
        void setID(double I, double D, double time);
        void setPID(double P, double I, double D);
        void setPID(double P, double I, double D, double time);

        double controlP(double err);
        double controlI(double err);
        double controlD(double err);
        double controlPI(double err);
        double controlPD(double err);
        double controlID(double err);
        double controlPID(double err);

        //To fill in your own measured velocity
        double controlR(double rate);
        double controlPR(double err, double rate);
        double controlIR(double err, double rate);
        double controlPIR(double err, double rate);

        //Same as above but for when we want to control 2 variables with the same k values
        //basically x and y
        Vector2 controlP2(Vector2 err);
        Vector2 controlI2(Vector2 err);
        Vector2 controlD2(Vector2 err);
        Vector2 controlPI2(Vector2 err);
        Vector2 controlPD2(Vector2 err);
        Vector2 controlID2(Vector2 err);
        Vector2 controlPID2(Vector2 err);

        //To fill in your own measured velocity
        Vector2 controlR2(Vector2 rate);
        Vector2 controlPR2(Vector2 err, Vector2 rate);
        Vector2 controlIR2(Vector2 err, Vector2 rate);
        Vector2 controlPIR2(Vector2 err, Vector2 rate);
    };

}

#endif //ROBOTEAM_AI_CONTROLLER_H
