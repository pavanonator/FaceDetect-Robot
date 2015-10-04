#ifndef __TACHI_DEFS_H__
#define __TACHI_DEFS_H__

#include <array>

// General Definitions //
#define NUM_LEGS    4
#define NUM_JOINTS  3
#define WAIST       0
#define THIGH       1
#define KNEE        2
#define WHEELS      3
#define UL          0
#define UR          1
#define DL          2
#define DR          3

// Device Ids //
#define WAIST_LEFT  1
#define WAIST_RIGHT 2
#define THIGH_UP    3
#define THIGH_LEFT  4
#define THIGH_RIGHT 5
#define THIGH_DOWN  6
#define KNEE_UL     7
#define KNEE_UR     8
#define KNEE_DL     9
#define KNEE_DR     10
#define WHEEL_UL    11
#define WHEEL_UR    12
#define WHEEL_DL    13
#define WHEEL_DR    14

// Coalesced Matrix Indeces //
#define WAIST_POS   0
#define THIGH_POS   1
#define KNEE_POS    2
#define WHEEL_VEL   3
#define WAIST_VEL   4
#define THIGH_VEL   5
#define KNEE_VEL    6

// Note: all the following measurements are in cm and radians
// for length and angle respectively

// waist parameters
const static double waist_x[4] = { -6.4, 6.4, -6.4, 6.4 };
const static double waist_y[4] = { 27.3, 27.3, -27.3, -27.3 };
const static double waist_angle[4] = { 1.309, -0.2618, 2.798, -1.8326 };
const static double waist_z = 4.0;
const static int waist_pot_min[4] = { 19, 19, 19, 19 };
const static int waist_pot_max[4] = { 53, 53, 53, 53 };

// thigh parameters
const static double thigh_length = 27.3;
const static int thigh_pot_min[4] = { 19, 19, 19, 19 };
const static int thigh_pot_max[4] = { 53, 53, 53, 53 };
const static double thigh_weight[4] = { 20.0, 20.0, 20.0, 20.0 };
const static double thigh_CM[4] = { 13.0, 13.0, 13.0, 13.0 };

// knee parameters
const static double knee_length = 43.1;
const static int knee_pot_min[4] = { 19, 19, 19, 19 };
const static int knee_pot_max[4] = { 53, 53, 53, 53 };
const static double knee_weight[4] = { 20.0, 20.0, 20.0, 20.0 };
const static double knee_CM[4] = { 18.0, 18.0, 18.0, 18.0 };

// conversion parameters
const static double pot_rad_ratio = 350.0;

#endif
