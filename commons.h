#ifndef COMMONS_H
#define COMMONS_H

int constrain(int a, int b, int c);
int dead_zone(int a, int b);
float constrain_f(float a, float b, float c);
float dead_zone_f(float a, float b);

#define GRAVITY 9810
#define VEHICLE_MASS 60
#define MAX_THRUST 100 //grams
#define DEG2RAD 0.01745
#define RAD2DEG 57.3
#define MAX_VELOCITY 2
#define CIRCLING_R 0.8
#define VICON_MARKER_DISTANCE 0.05

//enum enum_TOL_cmd{TakeOff, Land, Kill}

#define RADIUS_SQUARE 0.4
#define VEHICLE_SIZE 0.1//distance diagonal
#define VEHICLE_EDGE_THRESHOLD 0.065 
#define VEHICLE_DRIFT 0.3//depends on crazyflies maxium velocity
#define REVISE_WEIGHT 0.0001 //dependes on the absolute value of error
#define ABOUT_EDGE 0.5
#endif
