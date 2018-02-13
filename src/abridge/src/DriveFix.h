#ifndef DRIVE_FIX2_H
#define DRIVE_FIX2_H
#include "math.h"
#include <sys/time.h>
#include <iostream>

using namespace std;

class DriveFix{
    int refreshTime;

    long currentTime;
    long lastCheckTime;

    int *e_left;
    int *e_right;

    int prev_e_left;
    int prev_e_right;

    int *curr_v_left;
    int *curr_v_right;

    int *v_output_left;
    int *v_output_right;

    int max_e_val;

    int last_v_left;
    int last_v_right;

    int adjust_value_right;
    int adjust_value_left;

    bool maxRegisterd;
    bool terminalVelocityReached;

    public:

        DriveFix(int *e_left, int *e_right, int *curr_v_left, int *curr_v_right, int *v_output_left, int *v_output_right, int refreshTime) :
            e_left(e_left), e_right(e_right), curr_v_left(curr_v_left), curr_v_right(curr_v_right), v_output_left(v_output_left),
            v_output_right(v_output_right), refreshTime(refreshTime){
            currentTime = millis();
            lastCheckTime = millis();

            adjust_value_left = 0;
            adjust_value_right = 0;

            prev_e_left = 0;
            prev_e_right = 0;

            maxRegisterd = false;
            terminalVelocityReached = false;
            max_e_val = 1000;
        }


        // Compute if the output values need to be adjusted
        void compute();

        // Returns the current time in millis
        long millis(){
            struct timeval tp;
            gettimeofday(&tp, 0);
            long int ms = tp.tv_sec * 1000 + tp.tv_usec / 1000;
            return ms;
        }

};
#endif
