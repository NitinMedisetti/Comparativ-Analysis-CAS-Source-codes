#pragma once
#include <Arduino.h>
#ifndef RAD_TO_DEG
#define RAD_TO_DEG 57.2957795131f
#endif
#include "Display.h" 

class Navigation
{
public:
    Navigation() {}

    // 1. LOCAL CARTESIAN PATH (Meters)
    NavCommand computeLocalPathCommand(float currN, float currE, 
                                       float targetN, float targetE, 
                                       float currentYaw)
    {
        float dN = targetN - currN;
        float dE = targetE - currE;
        float targetHeading = atan2(dE, dN) * RAD_TO_DEG;
        if (targetHeading < 0) targetHeading += 360.0f;

        float error = targetHeading - currentYaw;
        while (error < -180.0f) error += 360.0f;
        while (error > 180.0f)  error -= 360.0f;

        if (abs(error) <= 8.0f) {
            return NAV_FORWARD;
        }
        
        if (abs(error) <= 8.0f) return NAV_FORWARD;
        if (error > 0) return NAV_TURN_RIGHT;
        return NAV_TURN_LEFT;
    }

    // 2. HEADING HOLD
    NavCommand computeHeadingCommand(float targetHeading, float currentYaw, bool allowForward)
    {
        float error = targetHeading - currentYaw;
        while (error < -180.0f) error += 360.0f;
        while (error > 180.0f)  error -= 360.0f;

        if (allowForward && abs(error) <= 8.0f) { //5
            return NAV_FORWARD;
        }

        if (abs(error) <= 8.0f) { //10
            if (allowForward) return NAV_FORWARD;
            return NAV_WAIT; 
        }

        if (error > 0) return NAV_TURN_RIGHT;
        return NAV_TURN_LEFT;
    }

    // 3. OBSTACLE AVOIDANCE (Updated Logic)
    NavCommand computeCommand(uint16_t frontUS, uint16_t leftUS, uint16_t rightUS)
    {
        const uint16_t US_FRONT_CLEAR = 60; //40 
        const uint16_t US_SIDE_BLOCK = 40; //30  

        if (frontUS > 0 && frontUS < US_FRONT_CLEAR) {
            if ((leftUS > 0 && leftUS <= US_SIDE_BLOCK)) return NAV_TURN_RIGHT;
            if ((rightUS > 0 && rightUS <= US_SIDE_BLOCK)) return NAV_TURN_LEFT;
            
            // Avoid oscillation if both free:
            // Only go Right if it is SIGNIFICANTLY larger than Left.
            // Otherwise default to Left.
            if (rightUS > leftUS + 20) return NAV_TURN_RIGHT;
            return NAV_TURN_LEFT;
        }

        return NAV_FORWARD;
    }
};