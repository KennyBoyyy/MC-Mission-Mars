#ifndef HANDLERS_H
#define HANDLERS_H

/**
 * This executable file will contain all the handlers and their functionality
 */

/**
 * @brief The SonarHandler class - this singleton class will be responsible for storing and
 * handling the sonar events
 */
class SonarHandler{
    double minDistance;
    bool isObstacleAvoidOn;
    static SonarHandler *s_instance;

    //private by default
    SonarHandler(){
        minDistance = 0.5; //min distance to call an avoid.
        isObstacleAvoidOn = true;
    }

    public:
        static SonarHandler* instance(){
            if(!s_instance)
                s_instance = new SonarHandler;
            return s_instance;
        }
};





















#endif
