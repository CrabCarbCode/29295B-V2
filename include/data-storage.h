#include "api.h"  //importing the motors and whatnot



#pragma region Variables

#pragma region PIDVariables // holds all variables required for the PID controller

// many of these are unneccesarily global / nonconstant, but I find the somewhat negligible innefficiencies
// to be worth the ease of understanding / workability, especially for those newer to robotics and programming

// control variables

bool drivePIDIsEnabled = false;
bool autonPIDIsEnabled = true;

int desiredDist;  // temporary
int desiredHeading;

// tuning coefficients

float lP;
float lD;
float lI;

float lO;

float rP;
float rD;
float rI;

float rO;

int integralBoundL;
int integralBoundR;

// Storage variables for Lateral (forward/back) PID

float previousErrorL = 0;  // position from last loop, MUST BE GLOBAL

float proportionalErrorL;  // reported value - desired value = position
float derivativeErrorL;    //(error - prevError)
float integralErrorL;

float lateralPower = 0;

// Storage variables for Rotational (turning) PID

float previousErrorR = 0;  // position from last loop, MUST BE GLOBAL

float proportionalErrorR;  // reported value - desired value = position
float derivativeErrorR;    //(error - prevError)
float integralErrorR;

float rotationalPower = 0;

// the second set of "rotational" storage vars above are technically useless, as the code executes in such a way that only one set of vars is needed
// to produce both outputs. however, readability is nice. change if memory ever becomes an issue

#pragma endregion  // end of PID variable declaration

// Autonomous Routing
struct autonCommand {
  // default values represent no change /input in each area

  float lateralDist = 0;
  float rotationalDist = 0;

  int8_t wingPattern = -1;
  int intakeSpeed = 0;

  int8_t armPosition = -1;
  int kickerSpeed = -1;

  float totalStepDelay = 0;
};


// Printing


autonCommand autonCommands[80];

// Timer

int ticksPerSec = 50;  // the number of 'ticks' in one second
int tickDeltaTime = 1000 / ticksPerSec;
int minPrintingDelay = (ticksPerSec / tickDeltaTime) + 0.5;  // ticksPerSec / tickDeltaTime

const int stepChangeCooldown = ticksPerSec / 4;  // sets the minimum delay between auton steps

// General Global Stuff

const float Pi = 3.14159265358;
const float e = 2.71828182845;

const int deadband = 3;  // if the controller sticks are depressed less than deadband%, input will be ignored (to combat controller drift)

const float autonDriveMult = 1.0;
// unused variable to increase / decrease speed of autonomous driving. just make a good drivetrain lol you'll be fine

int maxIntakeSpeed = 70;  // max intakeSpeed as a percent
int maxKickerSpeed = 60;

int armLevel = 1;
const int maxArmLevel = 2;

int currentPage = 1;
int globalTimer = 0;

const float degPerCM = (360 / (4.1875 * Pi * 2.54)) * (84.0f / 36.0);  // # of degrees per centimeter = 360 / (2Pir" * 2.54cm/") * gear ratio

#pragma endregion



#pragma region autonRoutes

int debugRoute(autonCommand currCommandList[]);


int rushDefenceRoute(autonCommand currCommandList[]);
int safeDefenceRoute(autonCommand currCommandList[]);


int rushOffenceRoute(autonCommand currCommandList[]);
int safeOffenceRoute(autonCommand currCommandList[]);


int fullSkillsRoute(autonCommand currCommandList[]);
int driverSkillsRoute(autonCommand currCommandList[]);

#pragma endregion

void SetPIDTunings(int range);

#pragma region printingConfigs

void selectorPrinting();

void autonPrinting();

void userControlPrinting();

#pragma endregion
