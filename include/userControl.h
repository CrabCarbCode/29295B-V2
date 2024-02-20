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
float ACurveExtremity = 0.1996;  // sigma
float peakPos = 1;               // mu
float AMinAmount = 0.24;         // kappa

// variables which control the shape of the stick curve
float linearHarshness = 0.6;  // g on graph
float SCurveExtremity = 5.3;  // h on graph



// tuning func
void tuneDrive(bool isPrinting);


// curve functions
float AccelSmoothingFunc(int time);

float StickSmoothingFunc(float stickVal);

// actual driving functions
void DrivingControl(bool isPrinting);

// handles user control input on intake / arm
void RCIntakeControls();

void KickerControl();

void ControlArm();

bool RWingLockedOut = false;
bool LWingLockedOut = false;

void WingsControl();