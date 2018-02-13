#include "DriveFix.h"

void DriveFix::compute(){
    // if the values are the same as last check
    if(last_v_left == *curr_v_left && last_v_right == *curr_v_right){

        if(!terminalVelocityReached){
            if(fabs(prev_e_left - *e_left) <= 30 || fabs(prev_e_right - *e_right) <= 30){
                terminalVelocityReached = true;

                maxRegisterd = true;
                if(*e_left > *e_right){
                    max_e_val = *e_left;
                } else {
                    max_e_val = *e_right;
                }

                lastCheckTime = millis();

            }
        } else {
            // if volts are equal and do not equal 0 (drive straight)
            if(*curr_v_left == *curr_v_right && *curr_v_left != 0){
                //if enough time passed to trigger a compute
                if(millis() - lastCheckTime >= refreshTime){
                    // If the dif between encoders is greater than 10 ticks
                    if(fabs(*e_left - *e_right) > 10){
                        // we need to adjust the slowest wheel
                        //if left or right encoders are greater than max
                        if(*e_left > max_e_val || *e_right > max_e_val){
                            // if left encoders is greater than max value
                            if(*e_left > max_e_val){
                                // lower the left voltage a bit
                                adjust_value_left -= 5;
                            }

                            //if right encoders is greater than max val
                            if(*e_right > max_e_val){
                                   // lower max
                                adjust_value_right -= 5;
                            }
                        }

                        //if left is greater than right
                        if(*e_left > *e_right && *e_right < max_e_val){
                            adjust_value_right += 5;

                        } else if(*e_right > *e_left && *e_left < max_e_val) {
                            adjust_value_left +=5;

                        }
                    }
                    lastCheckTime = millis();
                }
            } else { //turning

            }
        }


        // boost e_right value
        *v_output_right = *curr_v_right + adjust_value_right;
        // boost e_left value
        *v_output_left = *curr_v_left + adjust_value_left;

    } else {
        last_v_left = *curr_v_left;
        last_v_right = *curr_v_right;
        *v_output_right = *curr_v_right;
        *v_output_left = *curr_v_left;
        adjust_value_right = 0;
        adjust_value_left = 0;
        maxRegisterd = false;
        terminalVelocityReached = false;
        lastCheckTime = millis();
    }

    prev_e_left = *e_left;
    prev_e_right = *e_right;

}
