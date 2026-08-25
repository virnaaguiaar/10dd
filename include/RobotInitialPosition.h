#ifndef ROBOT_INITIAL_POSITION_H
#define ROBOT_INITIAL_POSITION_H

#include "Configs.h"
#include <ArduinoJson.h>

struct InitialPosition {
    float x;
    float y;
    float theta;
    bool isValid;
};

enum FormationType {
    FORMATION_SINGLE = 0,
    FORMATION_LINE,
    FORMATION_TRIANGLE,
    FORMATION_GRID_2X2,
    FORMATION_GRID_2X3,
    FORMATION_CROSS,
    FORMATION_CIRCLE,
    FORMATION_HEXAGON,
    FORMATION_SPIRAL
};

class RobotInitialPosition {
private:
    FormationType formation = FORMATION_SINGLE;
    int totalRobots = 1;
    float spacing = 0.6f;
    float circleRadius = 0.8f;
    
    int getRobotIndex(const String& robotId) {
        String numStr = robotId.substring(4);
        return numStr.toInt() - 1;
    }
    
public:
    void setFormationFromCount(int count) {
        totalRobots = count;
        if (count <= 1) formation = FORMATION_SINGLE;
        else if (count == 2) formation = FORMATION_LINE;
        else if (count == 3) formation = FORMATION_TRIANGLE;
        else if (count == 4) formation = FORMATION_GRID_2X2;
        else if (count <= 6) formation = FORMATION_GRID_2X3;
        else formation = FORMATION_CIRCLE;
    }
    
    void setFormation(FormationType type) { formation = type; }
    void setSpacing(float s) { spacing = s; }
    void setCircleRadius(float r) { circleRadius = r; }
    
    InitialPosition getInitialPosition(const String& robotId) {
        InitialPosition pos = {0, 0, 0, false};
        int index = getRobotIndex(robotId);
        
        if (index < 0 || index >= totalRobots) {
            pos.isValid = false;
            return pos;
        }
        
        pos.isValid = true;
        pos.theta = 0;
        
        switch (formation) {
            case FORMATION_SINGLE:
                pos.x = 0; pos.y = 0;
                break;
            case FORMATION_LINE:
                pos.x = (index - (totalRobots - 1) / 2.0f) * spacing;
                pos.y = 0;
                break;
            case FORMATION_TRIANGLE:
                if (index == 0) { pos.x = -spacing/2; pos.y = -spacing/3; }
                else if (index == 1) { pos.x = spacing/2; pos.y = -spacing/3; }
                else { pos.x = 0; pos.y = 2*spacing/3; }
                break;
            case FORMATION_GRID_2X2:
                pos.x = (index % 2 == 0 ? -0.5f : 0.5f) * spacing;
                pos.y = (index < 2 ? -0.5f : 0.5f) * spacing;
                break;
            case FORMATION_GRID_2X3:
                pos.x = (index % 2 == 0 ? -0.5f : 0.5f) * spacing;
                pos.y = (index / 2 - 1) * spacing;
                break;
            case FORMATION_CROSS:
                if (index == 0) { pos.x = 0; pos.y = 0; }
                else if (index == 1) { pos.x = -spacing; pos.y = 0; }
                else if (index == 2) { pos.x = spacing; pos.y = 0; }
                else if (index == 3) { pos.x = 0; pos.y = -spacing; }
                else { pos.x = 0; pos.y = spacing; }
                break;
            case FORMATION_CIRCLE: {
                float angle = (2 * PI * index) / totalRobots;
                pos.x = circleRadius * cos(angle);
                pos.y = circleRadius * sin(angle);
                pos.theta = angle;
                break;
            }
            case FORMATION_HEXAGON: {
                float angle = (2 * PI * index) / totalRobots - PI/2;
                float r = circleRadius * 0.8;
                pos.x = r * cos(angle);
                pos.y = r * sin(angle);
                pos.theta = angle + PI/2;
                break;
            }
            case FORMATION_SPIRAL: {
                float angle = (2 * PI * index) / totalRobots;
                float r = circleRadius * 0.2 + (index / (float)totalRobots) * circleRadius * 0.8;
                pos.x = r * cos(angle);
                pos.y = r * sin(angle);
                pos.theta = angle;
                break;
            }
            default:
                pos.x = 0; pos.y = 0;
        }
        
        return pos;
    }
    
    void publishFormationInfo() {
        StaticJsonDocument<512> doc;
        doc["formation_type"] = formation;
        doc["total_robots"] = totalRobots;
        doc["spacing"] = spacing;
        doc["robot_id"] = robotId;
        
        InitialPosition pos = getInitialPosition(String(robotId));
        if (pos.isValid) {
            doc["initial_x"] = pos.x;
            doc["initial_y"] = pos.y;
            doc["initial_theta"] = pos.theta;
        }
        
        char buffer[512];
        serializeJson(doc, buffer);
        // publishMQTT será chamado externamente
        if (mqttConnected) {
            extern void publishMQTT(const char*, const char*);
            publishMQTT("robot/formation", buffer);
        }
    }
};

extern RobotInitialPosition robotInitialPosition;

#endif