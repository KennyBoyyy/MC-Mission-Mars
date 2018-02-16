#include "DriveFix.h"

void DriveFix::compute(){    
    // if the values are the same as last check
    if(last_v_left == *curr_v_left && last_v_right == *curr_v_right && curr_v_right != 0 && curr_v_left != 0){
        if(!valuesMap.count(make_pair(*curr_v_left, *curr_v_right)) && (millis() - last_terminal_check_time) > 200){
            cout << "DRIVEFIXE: prev_eL: "<< prev_e_left << " prev_er: " <<prev_e_right<<endl;
            cout << "DRIVEFIXE: eL: "<< *e_left << " er: " <<*e_right<<endl;
            if(((prev_e_left != *e_left) && (fabs(fabs(prev_e_left) - fabs(*e_left)) <= 20)) || ((prev_e_right != *e_right) && (fabs(fabs(prev_e_right) - fabs(*e_right)) <= 20))){
                if(fabs(*e_left) > fabs(*e_right)){
                    max_e_val = *e_left;
                    cout<<"DRIVEFIX: terminal on left "<<*e_left<<endl;
                } else {
                    max_e_val = *e_right;
                    cout<<"DRIVEFIX: terminal on right "<<*e_right<<endl;
                }

                adjust_value_left = 0;
                adjust_value_right = 0;

                if(*curr_v_left == * curr_v_right)
                    valuesMap[make_pair(*curr_v_left, *curr_v_right)] = Point(adjust_value_left, adjust_value_right, max_e_val);

                lastCheckTime = millis();
                last_terminal_check_time = millis();

            } else {
                last_terminal_check_time = millis();
                adjust_value_left = 0;
                adjust_value_right = 0;
            }
        }

        if (valuesMap.count(make_pair(*curr_v_left, *curr_v_right))) {
            adjust_value_left = valuesMap[make_pair(*curr_v_left, *curr_v_right)].left;
            adjust_value_right = valuesMap[make_pair(*curr_v_left, *curr_v_right)].right;
            max_e_val = valuesMap[make_pair(*curr_v_left, *curr_v_right)].terminal;

            cout<<"DRIVEFIX: used learned value for left: "<<*curr_v_left << " for right: "<<*curr_v_right<<endl;

            // if volts are equal and do not equal 0 (drive straight)
            if(*curr_v_left == *curr_v_right && *curr_v_left != 0){
                //if enough time passed to trigger a compute
                if(millis() - lastCheckTime >= refreshTime){
                    // If the dif between encoders is greater than 10 ticks
                    if(fabs(*e_left - *e_right) > 5){
                        // we need to adjust the slowest wheel
                        //if left or right encoders are greater than max

                        //If driving forwards
                        if(curr_v_left > 0){
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
                        } else { // If driving backwards
                            if(*e_left < max_e_val || *e_right < max_e_val){
                                // if left encoders is greater than max value
                                if(*e_left < max_e_val){
                                    // lower the left voltage a bit
                                    adjust_value_left += 5;
                                }

                                //if right encoders is greater than max val
                                if(*e_right < max_e_val){
                                       // lower max
                                    adjust_value_right += 5;
                                }
                            }
                        }

                        //if left is greater than right
                        // In some cases ajusting will not help because we will be at the max speed of
                        // the motors. This is why there is an if statement checking that adjust is
                        // between -255 and 255

                        // If driving forwards
                        if(curr_v_left > 0){
                            if(*e_left > *e_right && *e_right < max_e_val){
                                if(adjust_value_right >= -255 && adjust_value_right <=255 )
                                    adjust_value_right += 5;

                            } else if(*e_right > *e_left && *e_left < max_e_val) {
                                if(adjust_value_left >= -255 && adjust_value_left <=255 )
                                    adjust_value_left +=5;

                            }
                        } else { // Driving backwards
                            if(*e_left > *e_right && *e_right < max_e_val){
                                if(adjust_value_right >= -255 && adjust_value_right <=255 )
                                    adjust_value_left -= 5;

                            } else if(*e_right > *e_left && *e_left < max_e_val) {
                                if(adjust_value_left >= -255 && adjust_value_left <=255 )
                                    adjust_value_right -=5;

                            }
                        }


                        cout<<"Learned new value for " << *curr_v_left <<endl;
                        //Save the currect setting.
                        valuesMap[make_pair(*curr_v_left, *curr_v_right)] = Point(adjust_value_left, adjust_value_right, max_e_val);
                    }
                    lastCheckTime = millis();
                }
            } /*else if(*curr_v_left == -(*curr_v_right) && *curr_v_left != 0) { //turning
                //if enough time passed to trigger a compute
                if(millis() - lastCheckTime >= refreshTime){
                    *e_left = fabs(*e_left);
                    *e_right = fabs(*e_right);
                    max_e_val = fabs(max_e_val);
                    // If the dif between encoders is greater than 10 ticks
                    if(fabs(*e_left - *e_right) > 5){
                        //If we are making a left turn
                        if(curr_v_left < 0){
                            // if left encoders is greater than max value
                            if(*e_left > max_e_val){
                                // lower the left voltage a bit
                                adjust_value_left += 5;
                            }

                            //if right encoders is greater than max val
                            if(*e_right > max_e_val){
                                   // lower max
                                adjust_value_right -= 5;
                            }
                        } else { // right turn
                            if(*e_left > max_e_val){
                                // lower the left voltage a bit
                                adjust_value_left -= 5;
                            }

                            //if right encoders is greater than max val
                            if(*e_right > max_e_val){
                                   // lower max
                                adjust_value_right += 5;
                            }
                        }


                        //If we are making a left turn
                        if(curr_v_left < 0){
                            if(*e_left > *e_right && *e_right < max_e_val){
                                if(adjust_value_right >= -255 && adjust_value_right <=255 )
                                    adjust_value_right += 5;

                            } else if(*e_right > *e_left && *e_left < max_e_val) {
                                if(adjust_value_left >= -255 && adjust_value_left <=255 )
                                    adjust_value_left -=5;

                            }
                        } else { //else if we are turning right
                            if(*e_left > *e_right && *e_right < max_e_val){
                                if(adjust_value_right >= -255 && adjust_value_right <=255 )
                                    adjust_value_right -= 5;

                            } else if(*e_right > *e_left && *e_left < max_e_val) {
                                if(adjust_value_left >= -255 && adjust_value_left <=255 )
                                    adjust_value_left +=5;

                            }
                        }

                        cout<<"Learned new value for " << *curr_v_left <<endl;
                        //Save the currect setting.
                        valuesMap[make_pair(*curr_v_left, *curr_v_right)] = Point(adjust_value_left, adjust_value_right, max_e_val);
                    }
                    lastCheckTime = millis();
                }
            }*/
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
        last_terminal_check_time = millis();


    }


    prev_e_left = *e_left;
    prev_e_right = *e_right;


}
