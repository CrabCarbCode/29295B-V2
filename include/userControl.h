#include "main.h"


int RAccelTime = 0;
int LAccelTime = 0;

int XStickStamp1 = 0;
int YStickStamp1 = 0;
int XStickStamp2 = 0;
int YStickStamp2 = 0;
int activeBrakeTimeStamp = 0;

int reverseDriveMult = 1;

// variables which control the shape/range of the acceleratory curve
float ACurveExtremity;  // sigma
float peakPos;          // mu
float AMinAmount;       // kappa

// variables which control the shape of the stick curve
float linearHarshness;  // g on graph
float SCurveExtremity;  // h on graph

// variables to track the state of the tuning routine
float adjustFactor = 0.05;  // the increment by which PID variables change during manual tuning
bool isTuningTurns = true;



// tuning func
void tuneDrive(bool isPrinting);


// curve functions
float AccelSmoothingFunc(int time);

float StickSmoothingFunc(float stickVal);

// actual driving functions
void DrivingControl(bool isPrinting);

// handles user control input on intake / arm
void RCIntakeControls();

void ControlArm();

bool RWingLockedOut = false;
bool LWingLockedOut = false;

void WingsControl();