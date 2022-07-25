/*
    User Config for OSSM - Reference board users should tweak this to match their personal build.
*/

/*
        Remote config
*/

uint8_t Remote_Address[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
//uint8_t Remote_Address[] = {0x08, 0x3A, 0xF2, 0x68, 0x1E, 0x74};

/*
        Motion System Config
*/

#define STEP_PER_REV      2000      // How many steps per revolution of the motor (S1 off, S2 on, S3 on, S4 off)
#define PULLEY_TEETH      20        // How many teeth has the pulley
#define BELT_PITCH        2         // What is the timing belt pitch in mm
#define MAX_RPM           3000.0    // Maximum RPM of motor
#define MAX_ACCELERATION  10000     // Maximum linear acceleration in mm/s²

// This is in millimeters, and is what's used to define how much of
// your rail is usable.
// The absolute max your OSSM would have is the distance between the belt attachments subtract
// the linear block holder length (75mm on OSSM)
#define MAX_STROKEINMM 180.0  // Real physical travel from one hard endstop to the other
#define STROKEBOUNDARY 10.0  // Safe distance the motion is constrained to avoiding crashes

// Calculation Aid:
#define STEP_PER_MM       STEP_PER_REV / (PULLEY_TEETH * BELT_PITCH)
#define MAX_SPEED         (MAX_RPM / 60.0) * PULLEY_TEETH * BELT_PITCH

#define DEPTH_RESULTION 5 // Depth Resultion in mm per encoder Klick
#define STROKE_RESULTION 5 // STROKE Resultion in mm per encoder Klick
#define ENCODER_RESULTION 36 // Klicks per turn
#define USER_SPEEDLIMIT 1200 // Speed in Cycles (in & out) per minute.

// The minimum value of the pot in percent
// prevents noisy pots registering commands when turned down to zero by user
const float commandDeadzonePercentage = 1.0f;
