#pragma region BoringVexConfigStuff

#include "main.h"

#include <math.h>    //neccessary for functions like abs() and round()
#include <stdlib.h>  //neccessary for std::[commands]

#include <cmath>
#include <cstring>
#include <sstream>  //neccessary for... logic
#include <string>   //neccessary for... using strings :sob:

#include "robot-config.h"  //importing the motors and whatnot

using namespace std;

#pragma endregion


// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // //


#pragma region NotesMaybeReadMe

// here to document weird quirks of V5, VSCode, or this particular program

/*

Sometimes things just break, like the abs() function was demanding 0 args.

Restarting the program fixed this Strings do not work in vex without external library shenanigans

This is my code, and thus it is my god given right to use it as a diary. ignore the strange comments
*/
#pragma endregion


// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // //


#pragma region GlobalVars

///// Control Variables //////


bool twoStickMode = true;  // toggles single or float stick drive
const int deadband = 3;    // if the controller sticks are depressed less than deadband%, input will be ignored (to combat controller drift)

const float autonDriveMult = 1.0;
// unused variable to increase / decrease speed of autonomous driving. just make a good drivetrain lol you'll be fine

const float Pi = 3.14159265358;
const float e = 2.71828182845;

int mStartPosL;
int mStartPosR;

int globalTimer = 0;
const int ticksPerSec = 50;  // the number of 'ticks' in one second
const int tickDeltaTime = 1000 / ticksPerSec;
int minPrintingDelay = (ticksPerSec / tickDeltaTime) + 0.5;  // ticksPerSec / tickDeltaTime

const float degPerCM = (360 / (4.1875 * Pi * 2.54)) * (84.0f / 36.0);  // # of degrees per centimeter = 360 / (2Pir" * 2.54cm/") * gear ratio

int maxIntakeSpeed = 70;  // max intakeSpeed as a percent
int armPos = 1;

int lastUpTimestamp = 0;
int lastDownTimestamp = 0;
int lastSpinTimestamp = 0;

int currentPage = 1;

#pragma endregion


// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // //


#pragma region HelperFunctions //unit conversions and whatnot

const char *toChar(std::string string) { return string.c_str(); }


int toInt(float val) { return val; }


int timeSincePoint(int checkedTime) {
  return checkedTime < globalTimer ? (globalTimer - checkedTime) : -1;  // returns -1 if checkedtime is in the future
}

template <typename T>
const float GreaterOf(T num1, T num2) {
  return (num1 > num2) ? num1 : num2;
}

const bool IsWithinRange(float num, float lowerBound, float upperBound) { return num >= lowerBound && num <= upperBound; }


// variables which control the shape/range of the acceleratory function
float ACurveExtremity = 0.1996;  // sigma
float peakPos = 1;               // mu
float AMinAmount = 0.24;         // kappa

// i have no idea what im doing
float AccelSmoothingFunc(int time) {  // returns a multiplier 0 to 1 based on time
  float x = time / 100;               // converting the input from percentage to a decimal btween 0-1

  const float multiplier =
      (0.5 / (ACurveExtremity * sqrt(2 * Pi))) * powf(e, (-0.5 * pow(((AMinAmount * x - AMinAmount * peakPos) / ACurveExtremity), 2)));
  return time >= (100) ? multiplier : 1;
}  // function graphed here: [https://www.desmos.com/calculator/y0fwlh6j47]


float linearHarshness = 0.6;  // g on graph
float SCurveExtremity = 5.3;  // h on graph

// i now have some idea what im doing
float StickSmoothingFunc(float stickVal) {
  float curveExponent = (abs(stickVal) - 100) / (10 * ACurveExtremity);
  float linearExponent = (-1 / (10 * linearHarshness));

  return (stickVal * (powf(e, linearExponent) + ((1 - powf(e, linearExponent)) * powf(e, curveExponent))));
}  // function graphed here: [https://www.desmos.com/calculator/ti4hn57sa7]

bool LDrive(float desPowerPercent) {
  LDrive600.move_velocity(desPowerPercent * 6);
  LDriveBackM.move_velocity(desPowerPercent * 2);

  // return true if motors are hitting desired speeds within 0.5%
  return IsWithinRange((LDriveFrontM.get_actual_velocity() - (6 * desPowerPercent)), -3, 3) ? true : false;
}

bool RDrive(float desPowerPercent) {
  RDrive600.move_velocity(desPowerPercent * 6);
  RDriveBackM.move_velocity(desPowerPercent * 2);

  // return true if motors are hitting desired speeds within 0.5%
  return IsWithinRange((RDriveFrontM.get_actual_velocity() - (6 * desPowerPercent)), -3, 3) ? true : false;
}

#pragma endregion


// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // //


#pragma region printingChicanery


void lcdControl() {
  if (globalTimer % 11 == 0) {  // refresh the screen every 11 ticks because 11 is a good number :)
    MainControl.clear();
  }

  if (MainControl.get_digital_new_press(DIGITAL_LEFT) && currentPage > 0) {
    currentPage--;
  }
  if (MainControl.get_digital_new_press(DIGITAL_RIGHT)) {
    currentPage++;
  }
}


bool isPrintingList[9] = {false, false, false, false, false, false, false, false, false};  // tracks which functions are trying to print
const int pagesPerPrint[9] = {1, 1, 3, 2, 2, 1, 2, 2, 2};  // hardcoded list containing the number of pages required for each function
/**
 * [index] [function] - [num of allocated pages]
 * [0] Rand Diagnostics - 1  //should not be used in final polished builds
 * [1] AutoSel - 1
 * [2] AutRoute - 2
 * [3] PID - 2
 * [4] Drivetrain - 2
 * [5] GPS - 1
 * [6] Kinematics - 2
 * [7] Drive Tune - 2
 * [8] PID Tune - 2
 **/

int pageRangeFinder(int index) {                 // calculates which page(s) a
  int startingPage = isPrintingList[0] ? 1 : 0;  // given function should print to
  // start on page 1
  for (int j = 0; j < index; ++j) {
    startingPage += isPrintingList[j] ? pagesPerPrint[j] : 0;
  }

  return startingPage;
}


void PrintToController(std::string prefix, double data, int numOfDigits, int row, int page) {  // handles single numbers
  if (currentPage == page && (globalTimer % 9 == (row * 3))) {
    std::string output = prefix + std::to_string(data).substr(0, numOfDigits + 1);
    // takes the first n digits of the number, adds it to output as string

    MainControl.print(row, 0, output.c_str(), 0);
  }
}


template <typename T, size_t N>
void PrintToController(std::string prefix, const std::array<T, N> &data, int numOfDigits, int row, int page) {  // handles multiple numbers
  if (currentPage == page && (globalTimer % 9 == (row * 3))) {
    std::string output = prefix;

    for (size_t j = 0; j < N; ++j) {
      double currNum = data[j];
      output += (j < N - 1) ? std::to_string(currNum).substr(0, numOfDigits + 1) + ", " : std::to_string(currNum).substr(0, numOfDigits + 1);
    }

    MainControl.print(row, 0, output.c_str(), 0);
  }
}


#pragma endregion


// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // //


#pragma region GPSAtHome

// this is both my magnum opus and the worst thing I've ever done i hate everything so so much

#pragma region relativeTracking


const float gAccel = 9.806;

std::array<double, 3> prevVelocity;

std::array<double, 3> calculateKinematics(bool isPrinting, bool getVelocity) {  // tracks displacement / acceleration / velocity relative to the robot

  const float deltaTime = tickDeltaTime / 1000;

  pros::c::imu_accel_s_t InertialAccelReading = Inertial.get_accel();  // checking the inertial is costly, so we do it once and capture the result
  std::array<double, 3> currAcceleration = {InertialAccelReading.x * gAccel, InertialAccelReading.y * gAccel, InertialAccelReading.z * gAccel};

  std::array<double, 3> currVelocity = {0.0, 0.0, 0.0};
  std::array<double, 3> distTravelled = {0.0, 0.0, 0.0};

  for (int i = 0; i < 3; i++) {  // tracks velocity / distance travelled over the current tick in all 3 axis

    currVelocity[i] = (currAcceleration[i] * deltaTime) + prevVelocity[i];
    distTravelled[i] = (prevVelocity[i] * deltaTime) + (0.5 * currAcceleration[i] * powf(deltaTime, 2));

    prevVelocity[i] = currVelocity[i];  // caches the velocity of the current tick for use in the next tick
  }

  if (isPrinting) {
    if (!isPrintingList[6]) {  // [6] Kinematics - 2
      isPrintingList[6] = true;
    }

    int startingPage = pageRangeFinder(6);

    PrintToController("Time: ", globalTimer, 5, 0, startingPage);
    PrintToController("Accel: ", currAcceleration, 3, 1, startingPage);
    PrintToController("cVel: ", currVelocity, 3, 2, startingPage);

    PrintToController("pVel: ", prevVelocity, 3, 1, startingPage + 1);
    PrintToController("Displ: ", distTravelled, 3, 2, startingPage + 1);
  }

  return getVelocity ? currAcceleration : distTravelled;
}


#pragma endregion



#pragma region globalTracking


std::array<double, 3> globalCoordinates;
std::array<double, 3> globalVelocities;
std::array<double, 3> totalDist;  // temporary, curious to see what just tracking displacement does

void updateGlobalPosition(bool isPrinting) {
  // capturing values of displacement and velocity over the last tick
  std::array<double, 3> currDisplacements = calculateKinematics(isPrinting, false);
  std::array<double, 3> currVelocities = calculateKinematics(isPrinting, true);


  for (int i = 0; i < 3; i++) {  // tracks displacement across all axis, slow and prolly doesn't work
    totalDist[i] += calculateKinematics(true, false).at(i);
  }

  // identify component of displacement change that should be added to each coordinate
  const float thetaHeading = (Inertial.get_heading() > 180) ? (Inertial.get_heading() - 360) : Inertial.get_heading();
  const float thetaPitch = (Inertial.get_pitch() > 180) ? (Inertial.get_pitch() - 360) : Inertial.get_pitch();
  const float thetaYaw = (Inertial.get_yaw() > 180) ? (Inertial.get_yaw() - 360) : Inertial.get_yaw();

  const float cosThetaHeading = cosf(thetaHeading);
  const float sinThetaHeading = sinf(thetaHeading);

  const float cosThetaPitch = cosf(thetaPitch);
  const float sinThetaPitch = sinf(thetaPitch);

  const float cosThetaYaw = cosf(thetaYaw);
  const float sinThetaYaw = sinf(thetaYaw);

  // decomposing the displacement vectors calculated from the inertial, then reconstructing them into the change in coordinates
  // this math REALLY fucking sucks, but I'm not sure theres a better / more efficient way to do this than hardcoding.
  globalCoordinates[0] += currDisplacements.at(0) * cosThetaHeading * cosThetaPitch  // x component of forward displacement
                          + currDisplacements.at(1) * sinThetaHeading * cosThetaYaw  // x component of sideways displacement
                          + currDisplacements.at(2) * sinThetaPitch * sinThetaYaw;   // x component of vertical displacement

  globalCoordinates[1] += currDisplacements.at(0) * sinThetaHeading * cosThetaPitch  // y component of forward displacement
                          + currDisplacements.at(1) * cosThetaHeading * cosThetaYaw  // y component of sideways displacement
                          + currDisplacements.at(2) * sinThetaPitch * sinThetaYaw;   // y component of vertical displacement

  globalCoordinates[2] += currDisplacements.at(0) * sinThetaPitch                   // z component of forward displacement
                          + currDisplacements.at(1) * sinThetaYaw                   // z component of sideways displacement
                          + currDisplacements.at(2) * cosThetaPitch * cosThetaYaw;  // z component of vertical displacement

  if (isPrinting) {
    if (!isPrintingList[5]) {  // [5] GPS - 1
      isPrintingList[5] = true;
    }

    int startingPage = pageRangeFinder(5);

    PrintToController("Time: ", globalTimer, 3, 0, startingPage);
    PrintToController("Displ: ", currDisplacements, 3, 1, startingPage);
    PrintToController("Coords: ", globalCoordinates, 3, 2, startingPage);

    PrintToController("Heading: %d", thetaHeading, 4, 0, 4);  // print angles
    PrintToController("Pitch: %d", thetaPitch, 1, 4, 4);
    PrintToController("Yaw: %d", thetaYaw, 2, 4, 4);
  }
}


#pragma endregion

#pragma endregion


// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // //


#pragma region PID //the code behind the autonomous Proportional Integral Derivative controller

#pragma region PIDVariables // holds all variables required for the PID controller

// many of these are unneccesarily global / nonconstant, but I find the somewhat negligible innefficiencies
// to be worth the ease of understanding / workability, especially for those newer to robotics and programming

// control variables

bool drivePIDIsEnabled = false;
bool autonPIDIsEnabled = true;

int desiredDist;  // temporary
int desiredHeading;

// tuning coefficients

float lP = 0.65;
float lD = 0.25;
float lI = 0.35;

float lOutput = 1.9;

float rP = 0.80;
float rD = 0.55;
float rI = 1.70;

float rOutput = 2.0;

int integralBoundL = 10 * degPerCM;
int integralBoundR = 5;

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

// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // //

int stuckTimeStamp = 0;
int avgMotorPosition = 0;

bool AutonPID(bool isPrinting) {
  if (autonPIDIsEnabled) {  // toggle so the PID can be disabled while placed on a separate thread
    // sets currHeading from -180 < h < 180, meaning we turn the correct direction from error
    const float absHeading = fabs(std::fmod(Inertial.get_heading(), 360.0f));
    const float currHeading = (absHeading > 180) ? absHeading - 360 : absHeading;

    const float absDesHead = std::fmod(desiredHeading, 360.0f);
    const float desHead = (absDesHead > 180) ? std::fmod((absDesHead - 360), 180) : std::fmod((absDesHead), 180);


    ///////////////////////////////////////
    //////        Lateral PID        //////
    ///////////////////////////////////////

    avgMotorPosition = ((RDriveFrontM.get_position()) + (LDriveFrontM.get_position())) / 2;

    proportionalErrorL = lP * (desiredDist - avgMotorPosition);     // proportional error
    derivativeErrorL = lD * (proportionalErrorL - previousErrorL);  // derivative of error

    // filters out the integral at short ranges (no I if |error| < constant lower limit, eg. 10cm),
    // allowing it to be useful when needed without overpowering other elements
    if (fabs(proportionalErrorL) > integralBoundL) {
      integralErrorL += lI * (derivativeErrorL);
    } else {
      integralErrorL = 0;
    }

    lateralPower = (proportionalErrorL + derivativeErrorL + integralErrorL) * lOutput;

    ///////////////////////////////////////
    //////      Rotational PID       //////
    ///////////////////////////////////////

    proportionalErrorR = rP * (desHead - currHeading);              // proportional error
    derivativeErrorR = rD * (proportionalErrorR - previousErrorR);  // derivative of error

    // filters out the integral at short ranges (no I if |error| < constant lower limit, eg. 10cm),
    // allowing it to be useful when needed without overpowering other elements
    if (fabs(proportionalErrorR) > integralBoundR) {
      integralErrorR += rI * (derivativeErrorR);
    } else {
      integralErrorR = 0;
    }

    rotationalPower = (proportionalErrorR + derivativeErrorR + integralErrorR) * rOutput;

    ///////////////////////////////////////
    //////        ending math        //////
    ///////////////////////////////////////

    LDrive(autonDriveMult * (lateralPower + rotationalPower));
    RDrive(autonDriveMult * (lateralPower - rotationalPower));

    previousErrorL = proportionalErrorL;
    previousErrorR = proportionalErrorR;

    if (isPrinting) {
      if (!isPrintingList[3]) {  // [3] PID - 2
        isPrintingList[3] = true;
      }

      int startingPage = pageRangeFinder(3);

      PrintToController("Time: ", globalTimer, 5, 0, startingPage);
      PrintToController("LOut: ", (autonDriveMult * (lateralPower + rotationalPower)), 5, 1, startingPage);
      PrintToController("ROut: ", (autonDriveMult * (lateralPower + rotationalPower)), 5, 2, startingPage);

      PrintToController("LError: ", proportionalErrorL, 5, 0, startingPage + 1);
      PrintToController("LatOut: ", lateralPower, 5, 1, startingPage + 1);
      PrintToController("RotOut: ", rotationalPower, 5, 2, startingPage + 1);
    }
  }

  return (fabs(proportionalErrorL) <= (3 * degPerCM) && fabs(proportionalErrorR) <= 1.5) ? true : false;  // return true if done movement
}


#pragma endregion  // end of PID block


// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // //


#pragma region ActualCompetitionFunctions

#pragma region AutonomousFunctions //Functions for autonomous control

int selectorStage = 0;
int selectedRoute = 3;


const int stepChangeCooldown = ticksPerSec / 4;  // sets the minimum delay between auton steps

// initializing data variables (used to track details of each step)
int autonStep = 0;  // tracks which step of the auton the program is on
int flywheelSpeed = 0;
int minStepChangeTimeStamp;

vector<float> autonCommands[50];

std::array<int, 6> ReadAutonStep(int currStep) {  // this entire function is kinda unneccessary but it looks nice
  vector<float> currCommand = autonCommands[currStep];

  int desDistCM = currCommand.at(0);
  int desHead = currCommand.at(1);

  int flystickPos = currCommand.at(2);
  int flywheelSpeed = currCommand.at(3);

  bool wingsOut = currCommand.at(4);

  int endDelayInTicks = currCommand.at(5) * ticksPerSec;

  return {desDistCM, desHead, flystickPos, flywheelSpeed, wingsOut, endDelayInTicks};
}

bool ArmUp = false;  // check if arm start down

bool ManageArm(bool isPrinting) {}

#pragma endregion  // end of AutonFunctions

#pragma region UserControlFunctions //handles all functions involving user input

int RAccelTime = 0;
int LAccelTime = 0;

int XStickStamp1 = 0;
int YStickStamp1 = 0;
int XStickStamp2 = 0;
int YStickStamp2 = 0;
int activeBrakeTimeStamp = 0;

int reverseDriveMult = 1;

void DrivingControl(bool isPrinting) {  // resoponsible for user control of the drivetrain

  switch (MainControl.get_digital_new_press(DIGITAL_L2) - MainControl.get_digital_new_press(DIGITAL_R2)) {
    case -1:
      reverseDriveMult = -1;
    case 0:
      break;
    case 1:
      reverseDriveMult = 1;
      break;
  }

  // taking the position of the sticks and appplying gradient diffusion to them. Check the StickSmoothingFunc graph for details
  // X stick covers fwd/back, Y stick covers turning

  float XStickPercent = StickSmoothingFunc(MainControl.get_analog(E_CONTROLLER_ANALOG_LEFT_Y) / 1.27 * reverseDriveMult);  // w on graph
  float YStickPercent = StickSmoothingFunc(MainControl.get_analog(E_CONTROLLER_ANALOG_RIGHT_X) / 1.27);                    // s on graph

  static float ptsPerTick = 4;

  // filter out stick drift / nonpressed sticks. saves resources by skipping calculations when not driving
  if ((abs(XStickPercent) + abs(YStickPercent)) >= deadband) {
    int fullStopThreshold = 150;

    // inreasing or decreasing the acceleration functions' timer
    LAccelTime += ((LAccelTime <= 100 && XStickPercent > deadband) || LAccelTime < 0) ? ptsPerTick : -ptsPerTick;  // Y(x) on graph
    RAccelTime += ((RAccelTime <= 100 && YStickPercent > deadband) || RAccelTime < 0) ? ptsPerTick : -ptsPerTick;  // X(x) on graph


    // applying the acceleratory curve to the stick inputs, multiplies the stick values by the output of the accel smoothing function

    int lateralOutput = AccelSmoothingFunc(LAccelTime) * XStickPercent;

    float rotationalMult = ((-0.001 * powf(lateralOutput, 2)) + (-0.25 * lateralOutput) + 100) / 100;
    // graphed and explained here: [https://www.desmos.com/calculator/03mizqcj4f]

    int rotationalOutput = (rotationalMult * AccelSmoothingFunc(RAccelTime) * YStickPercent);  // ((100 - abs(lateralOutput)) / 100)

    // converting the fwd/bckwd/turning power into output values for the left and right halves of the drivetrain, then driving

    int leftOutput = ((lateralOutput + rotationalOutput));
    int rightOutput = ((lateralOutput - rotationalOutput));

    bool latFullStop = ((abs(XStickStamp1 - XStickPercent) > fullStopThreshold) || (abs(XStickStamp2 - XStickPercent) > fullStopThreshold));
    bool rotFullStop = ((abs(YStickStamp1 - XStickPercent) > fullStopThreshold) || (abs(YStickStamp2 - XStickPercent) > fullStopThreshold));

    // implementing hard stops if sticks are flicked the opposite way
    if (latFullStop || rotFullStop) {
      LDrive(leftOutput);  // slightly moves in the new stick direction, opposite to the previous
      RDrive(rightOutput);

      activeBrakeTimeStamp = globalTimer;
    } else if (timeSincePoint(activeBrakeTimeStamp) <= 3) {  // brake for a fraction of a second to kill momentum
      LDrive(0);
      RDrive(0);
    } else {
      LDrive(0.95 * leftOutput);  // stepping up the output from 0-100% to 0-600rpm with a little headroom
      RDrive(0.95 * rightOutput);
    }

    if (globalTimer % 6) {
      XStickStamp1 = XStickPercent;
      YStickStamp1 = YStickPercent;
    } else if (globalTimer % 6 == 3) {
      XStickStamp2 = XStickPercent;
      YStickStamp2 = YStickPercent;
    }

    if (isPrinting) {  // [4] Drivetrain - 2
      if (!isPrintingList[4]) {
        isPrintingList[4] = true;
      }

      int startPage = pageRangeFinder(4);

      PrintToController("Time: ", globalTimer, 5, 0, startPage);
      PrintToController("Reversed?: ", reverseDriveMult, 5, 1, startPage);
      PrintToController("Accel: ", LAccelTime, 4, 2, startPage);

      PrintToController("OutAdj: ", rotationalMult, 5, 0, startPage + 1);
      PrintToController("LOut: ", lateralOutput, 5, 1, startPage + 1);
      PrintToController("ROut: ", rotationalOutput, 5, 2, startPage + 1);
    }


  } else {  // if not want move, dont move
    LDrive(0);
    RDrive(0);

    LAccelTime -= (LAccelTime > 0) ? ptsPerTick : -ptsPerTick;
    RAccelTime -= (RAccelTime > 0) ? ptsPerTick : -ptsPerTick;

    if (isPrinting) {  // [4] Drivetrain - 2
      if (!isPrintingList[4]) {
        isPrintingList[4] = true;
      }

      int startPage = pageRangeFinder(4);

      PrintToController("Time: ", globalTimer, 5, 0, startPage);
      PrintToController("Reversed?: ", reverseDriveMult, 5, 1, startPage);
      PrintToController("Accel: ", LAccelTime, 4, 0, startPage);

      PrintToController("OutAdj: ", 0.0, 1, 0, startPage + 1);
      PrintToController("LOut: ", 0.0, 1, 1, startPage + 1);
      PrintToController("ROut: ", 0.0, 1, 2, startPage + 1);
    }
  }
}  // graphed and simulated at [https://www.desmos.com/calculator/4dse2rfj], modelled in % power output by default. Graph may be outdated


#pragma region AuxiliaryFunctions

// handles user control of intake
void RCIntakeControls() { IntakeM.move_velocity(maxIntakeSpeed * 6 * (MainControl.get_digital(DIGITAL_L1) - MainControl.get_digital(DIGITAL_R1))); }

bool RWingLockedOut = false;
bool LWingLockedOut = false;

void WingsControl() {  // controls... wings

  if (MainControl.get_digital_new_press(DIGITAL_Y)) {
    LWingLockedOut = !LWingLockedOut;
    WingPL.set_value(LWingLockedOut);
  }

  if (MainControl.get_digital_new_press(DIGITAL_B)) {
    RWingLockedOut = !RWingLockedOut;
    WingPR.set_value(RWingLockedOut);
  }
}

#pragma endregion  // end of AuxilaryFunctions

#pragma endregion  // end of UserControlFunctions

#pragma endregion  // end of Bot controlling functions


// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // //


#pragma region debugFunctions

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////                                                                                                                 ////
//                                                 PID TUNING INSTRUCTIONS:                                            //
//   1. call the tunePID function in opcontrol()                                                                       //
//   2. confirm that the P/I/D tuning variables in the "PIDVariables" region are set to 0.0, with the outputs at 1.0   //
//   3. follow the control layout found here: [http://tinyurl.com/3zrb6zj5]                                            //
//   4. increase the lP/rP coefficient(s) until the desired motion is completed with oscilations                       //
//   5. increase the lD/rD coefficient(s) until the oscilations dampen out over time                                   //
//   6. increase the lI/rI coefficient(s) until the motion is completed aggressively without oscilations               //
//      (somewhat optional, as not all applications benefit from an integral controller)                               //
//   7. increase the output coefficient(s) until the motion is completed with acceptable speed and precision           //
//                                                                                                                     //
//      as builds and use cases vary, you may need to fiddle with the values after initial tuning after more testing.  //
//      generally the P & D components should be larger than the I, and values should be between 0.0 and 5.0.          //
////                                                                                                                 ////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float adjustFactor = 0.05;  // the increment by which PID variables change during manual tuning
bool isTuningTurns = true;

void tunePID(bool isPrinting) {  // turns or oscilates repeatedly to test and tune the PID, allowing real-time tuning and adjustments

  lP = 0.5;
  lD = 0.0;
  lI = 0.0;

  lOutput = 1.0;

  rP = 0.5;
  rD = 0.0;
  rI = 0.0;

  rOutput = 1.0;

  desiredDist = 0 * degPerCM;
  desiredHeading = 0;

  while (true) {
    lcdControl();

    if (MainControl.get_digital_new_press(DIGITAL_B)) {  // changes proportional coefficient
      rP += isTuningTurns ? adjustFactor : 0;
      lP += isTuningTurns ? 0 : adjustFactor;
    }
    if (MainControl.get_digital_new_press(DIGITAL_A)) {  // changes integral coefficient
      rI += isTuningTurns ? adjustFactor : 0;
      lI += isTuningTurns ? 0 : adjustFactor;
    }
    if (MainControl.get_digital_new_press(DIGITAL_Y)) {  // changes derivative coefficient
      rD += isTuningTurns ? adjustFactor : 0;
      lD += isTuningTurns ? 0 : adjustFactor;
    }


    if (MainControl.get_digital_new_press(DIGITAL_X)) {  // toggles increases/decreases to tuning variables
      adjustFactor *= -1;
    }
    if (MainControl.get_digital_new_press(DIGITAL_UP)) {  // toggles between testing rotational / lateral drive
      isTuningTurns = !isTuningTurns;
    }
    if (MainControl.get_digital_new_press(DIGITAL_R1)) {  // changes output power of lateral PID
      lOutput += adjustFactor;
    }
    if (MainControl.get_digital_new_press(DIGITAL_L1)) {  // changes output power of rotational PID
      rOutput += adjustFactor;
    }


    bool stepDone = AutonPID(true);

    // flips 180 or drives 1m in alternating directions at regular intervals
    if (globalTimer % (6 * ticksPerSec) < (3 * ticksPerSec)) {
      desiredDist = isTuningTurns ? 0 : 15 * degPerCM;
      desiredHeading = isTuningTurns ? 25 : 0;
    } else {
      desiredDist = isTuningTurns ? 0 : -15 * degPerCM;
      desiredHeading = isTuningTurns ? -25 : 0;
    }


    if (isPrinting) {
      if (!isPrintingList[8]) {  // [8] PID Tune - 2
        isPrintingList[8] = true;
      }


      int startPage = pageRangeFinder(8);

      PrintToController("PVar: ", (isTuningTurns ? rP : lP), 4, 0, startPage);
      PrintToController("IVar: ", (isTuningTurns ? rI : lI), 4, 1, startPage);
      PrintToController("DVar: ", (isTuningTurns ? rD : lD), 4, 2, startPage);

      PrintToController("Turning?: ", isTuningTurns, 1, 0, startPage + 1);
      PrintToController("lOutput: ", lOutput, 2, 1, startPage + 1);
      PrintToController("rOutput: ", rOutput, 2, 2, startPage + 1);
    }


    globalTimer++;
    delay(tickDeltaTime);
  }
}



void tuneDrive(bool isPrinting) {  // allows for user driving, with real time control over drive coefficients
  // default settings for acceleratory / stick curves

  isPrintingList[7] = true;  // Drive Tuning - disabled

  ACurveExtremity = 0.19948;  // sigma
  AMinAmount = 0.235;         // kappa

  linearHarshness = 0.2;  // g on graph
  SCurveExtremity = 4.7;  // h on graph

  float adjustFactor = 1;

  while (true) {
    lcdControl();

    if (MainControl.get_digital_new_press(DIGITAL_X)) {
      ACurveExtremity += adjustFactor / 100000;
    }
    if (MainControl.get_digital_new_press(DIGITAL_A)) {
      AMinAmount += adjustFactor / 1000;
    }
    if (MainControl.get_digital_new_press(DIGITAL_B)) {
      linearHarshness += adjustFactor / 20;
    }
    if (MainControl.get_digital_new_press(DIGITAL_Y)) {
      SCurveExtremity += adjustFactor / 10;
    }

    if (MainControl.get_digital_new_press(DIGITAL_UP)) {
      adjustFactor++;
    }
    if (MainControl.get_digital_new_press(DIGITAL_DOWN)) {
      adjustFactor--;
    }
    if (MainControl.get_digital_new_press(DIGITAL_L1)) {
      adjustFactor *= -1;
    }

    DrivingControl(true);


    if (isPrinting) {  // [7] Drive Tune - 1
      if (!isPrintingList[7]) {
        isPrintingList[7] = true;
      }

      int startPage = pageRangeFinder(7);

      PrintToController("kappa: ", ACurveExtremity, 7, 0, startPage);
      PrintToController("sigma: ", AMinAmount, 5, 1, startPage);

      std::array<float, 2> HAndG = {linearHarshness, SCurveExtremity};
      PrintToController("g and h: ", HAndG, 3, 2, startPage);
    }


    globalTimer++;
    delay(tickDeltaTime);
  }
}

#pragma endregion


// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // //


#pragma region dataStorage // should be ported to individual header files at some point

#pragma region autonRoutes

int totalNumOfCommands;

void skillsAuton() {
  // autonCommands[ autonStep ] = { latDist(cm), rotDist(degrees), armPos(1-5, 0 = no change), flywheelSpeed(%), wings[vvv], delay(sec) }
  // wings: 0 = none, 1 = both, 2 = right, 3 = left

  autonCommands[0] = {0, 0, 0, 0, 0, 0};      // [null padding, [DO NOT REMOVE] start parallel to match loading bar, front facing wall
  autonCommands[1] = {-55, 0, 0, 0, 0, 0};    // back along the match loading bar
  autonCommands[2] = {-10, 45, 0, 0, 0, 0};   // turn so back faces net [distance]***
  autonCommands[3] = {-30, 0, 0, 0, 0, 0};    // ram triballs under net
  autonCommands[4] = {37, 0, 0, 0, 0, 0};     // pull out of net, almost to match-load bar
  autonCommands[5] = {5, -90, 0, 0, 0, 0};    // turn 90 to face target (close net corner)
  autonCommands[6] = {-8, -14, 3, 0, 0, 0};   // drive back to barely touch bar [distance]***, raise arm
  autonCommands[7] = {0, 0, 0, 90, 0, 1};     // match load, 20 seconds?
  autonCommands[8] = {10, 0, 0, 0, 0, 0};     // fwd from match load bar
  autonCommands[9] = {0, -130, 0, 0, 0, 0};   // turn so back is towards gutter
  autonCommands[10] = {82, 0, 0, 0, 0, 0};    // drive into mouth of gutter
  autonCommands[11] = {16, -38, 0, 0, 0, 0};  // turn to face gutter
  autonCommands[12] = {105, 0, 0, 0, 0, 0};   // drive down length of gutter
  autonCommands[13] = {61, 0, 0, 0, 0, 0};    // drive down length of gutter
  autonCommands[14] = {0, -45, 0, 0, 2, 0};   // open right wing to clear ML corner, turn towards net
  autonCommands[15] = {61, 0, 0, 0, 2, 0};    // drive bckwds past ML bar
  autonCommands[16] = {10, -45, 0, 0, 0, 0};  // close wings, turn to face net
  autonCommands[17] = {20, 0, 0, 0, 0, 0};    // ram balls into bar
  autonCommands[18] = {-30, 0, 0, 0, 0, 0};   // re-ram pt.1 (out)
  autonCommands[19] = {30, 0, 0, 0, 0, 0};    // re-ram pt.2 (in)
  autonCommands[20] = {-26, 0, 0, 0, 0, 0};   // pull out of net
  autonCommands[21] = {0, 94, 0, 0, 0, 0};    // turn to face middle bar
  autonCommands[22] = {-110, 0, 0, 0, 0, 0};  // drive almost up to middle bar
  autonCommands[23] = {0, -46, 0, 0, 1, 0};   // open wings, turn to face net at angle
  autonCommands[24] = {85, 0, 0, 0, 1, 0};    // ram orbs into net
  autonCommands[25] = {-30, 0, 0, 0, 2, 0};   // re-ram pt.1 (out) (close left wing to avoid pushing triballs away from net)
  autonCommands[26] = {30, 0, 0, 0, 1, 0};    // re-ram pt.2 (in)
  autonCommands[27] = {-75, 0, 0, 0, 0, 0};   // close wings, back up
  autonCommands[28] = {0, -45, 0, 0, 0, 0};   // turn to be in line with middle bar
  autonCommands[29] = {50, 0, 0, 0, 3, 0};    // open wings ( only left wing would be nice, should implement), drive forward
  autonCommands[30] = {0, 45, 0, 0, 3, 0};    // turn to be facing net (angled towards middle)
  autonCommands[31] = {85, 0, 0, 0, 1, 0};    // ram orbs into net, open both wings
  autonCommands[32] = {-30, 0, 0, 0, 2, 0};   // re-ram pt.1 (out), (close left wing to avoid pushing triballs away from net)
  autonCommands[33] = {30, 0, 0, 0, 1, 0};    // re-ram pt.2 (in),
  autonCommands[34] = {-65, 0, 0, 0, 0, 0};   // close wings, back up most of the way
  autonCommands[35] = {0, -115, 0, 0, 0, 0};  // turn to be facing corner, more towards gutter
  autonCommands[36] = {45, 0, 0, 0, 0, 0};    // drive towards corner
  autonCommands[37] = {0, -50, 0, 0, 0, 0};   // turn so back is facing net
  autonCommands[38] = {-45, 0, 0, 0, 1, 0};   // open wings, move along ML bar
  autonCommands[39] = {0, -45, 0, 0, 0, 0};   // close wings, turn back directly towards net
  autonCommands[40] = {110, 0, 0, 0, 0, 0};   // ram orbs into net
  autonCommands[41] = {0, -90, 0, 0, 0, 0};   // re-ram pt.1 (out)
  autonCommands[42] = {110, 0, 0, 0, 0, 0};   // re-ram pt.2 (in)
  autonCommands[43] = {0, -90, 0, 0, 0, 0};   // move away
  autonCommands[44] = {100, 0, 0, 0, 0, 0};   // Turn to go down
  autonCommands[45] = {0, -90, 0, 0, 0, 0};   // Turn to go down
  autonCommands[46] = {30, 0, 0, 0, 0, 0};    // Turn to go down
  autonCommands[47] = {-30, 0, 0, 0, 0, 0};   // Turn to go down
  autonCommands[48] = {30, 0, 0, 0, 0, 0};    // Turn to go down
  autonCommands[49] = {-60, 0, 0, 0, 0, 0};   // Turn to go down

  // samich yummmmmmmmmy

  totalNumOfCommands = 50;  // manual verification that the num of steps is correct (last num in array + 1)
  autonCommands->resize(totalNumOfCommands);
}

void offenceAuton() {  // starting on the enemy side of the field (no match loading)
  // autonCommands[ autonStep ] = { latDist(cm), rotDist(degrees), armPos(1-5, 0 = no change), flywheelSpeed(%), wings[vvv], delay(sec) }
  // wings: 0 = none, 1 = both, 2 = right, 3 = left

  // safe route, does not try to score ball under horizontal bar. If have extra time, go for unsafe route?
  autonCommands[0] = {0, 0, 0, 0, 0, 0};      // [null padding, DO NOT REMOVE] start touching match load bar, facing towards middle triballs
  autonCommands[1] = {10, 0, 0, 0, 0, 0};     // move slightly away from ML bar
  autonCommands[2] = {0, 45, 0, 0, 0, 0};     // smack preload towards net
  autonCommands[3] = {0, -45, 0, 0, 0, 0};    // course correct towards triball
  autonCommands[4] = {85, 0, 2, 0, 0, 0};     // drive close to lower middle triball
  autonCommands[5] = {10, 0, 3, 50, 0, 0};    // raise arm and drive to be barely touching middle triball with intake bands, spin flywheel
  autonCommands[6] = {0, 110, 2, 0, 0, 0};    // lower arm, spin to be facing our net (slightly away from middle of field to prevent interference)**
  autonCommands[7] = {0, 0, 0, -60, 0, 0};    // outtake orb
  autonCommands[8] = {0, -200, 0, 0, 0, 0};   // turn to directly face upper middle orb
  autonCommands[9] = {45, 0, 0, 0, 0, 0};     // drive close to orb
  autonCommands[10] = {10, 0, 3, 50, 0, 0};   // drive such that bands touch orb, raise arm
  autonCommands[11] = {5, 0, 2, 0, 0, 0};     // capture orb
  autonCommands[12] = {-10, 0, 0, 0, 0, 0};   // back up slightly
  autonCommands[13] = {0, 200, 0, 0, 0, 0};   // turn to face net (also slightly away from middle)
  autonCommands[14] = {0, 0, 0, -60, 0, 0};   // outtake orb
  autonCommands[15] = {80, 0, 0, 0, 2, 0};    // ram orbs under net with only right wing, to avoid risk of hitting opponents
  autonCommands[16] = {-40, 0, 0, 0, 0, 0};   // back away from net
  autonCommands[17] = {0, 145, 0, 0, 0, 0};   // turn towards corner triball
  autonCommands[18] = {60, 0, 0, 0, 0, 0};    // get close to orb
  autonCommands[19] = {10, 0, 3, 50, 0, 0};   // raise arm, creep towards orb & prep ''intake''
  autonCommands[20] = {5, 0, 2, 0, 0, 0};     // capture orb
  autonCommands[21] = {0, -130, 0, 0, 0, 0};  // turn to face space between ML bar and side of net
  autonCommands[22] = {50, 0, 0, 0, 0, 0};    // drive until past the leg of the black horizontal bar
  autonCommands[23] = {0, 0, 0, -75, 0, 0};   // bowl orb to be in front of the net's side *would be preferable to keep in intake entire time
  // not sure outtaking is consistent enough for that though
  autonCommands[24] = {0, 15, 0, 0, 0, 0};   // turn to face ML bar
  autonCommands[25] = {35, 0, 0, 0, 0, 0};   // drive almost up to ML bar
  autonCommands[26] = {0, -90, 0, 0, 2, 0};  // turn to be parallel with ML bar, extend wing
  autonCommands[27] = {20, 0, 0, 0, 0, 0};   // clear corner orb
  autonCommands[28] = {0, -45, 0, 0, 0, 0};  // turn to face net (this has us ram side of net with front of bot, may not have enough time to flip)
  autonCommands[29] = {45, 0, 0, 0, 0, 0};   // ram bowled ball, preload and corner ball into net
  autonCommands[30] = {0, 0, 0, 0, 0, 0};    // needs to navigate back to bar for AWP but im eepy

  totalNumOfCommands = 31;
  autonCommands->resize(totalNumOfCommands);
}

void defenceAuton() {  // starting on the team side of the field (match loading)
  // autonCommands[ autonStep ] = { latDist(cm), rotDist(degrees), armPos(1-5, 0 = no change), flywheelSpeed(%), wings[vvv], delay(sec) }
  // wings: 0 = none, 1 = both, 2 = right, 3 = left

  // 4 ball over, preload under defense auton
  autonCommands[0] = {0, 0, 0, 0, 0, 0};      // [null padding, DO NOT REMOVE] start parallel to match loading bar, front facing wall (skills setup)
  autonCommands[1] = {-55, 0, 2, 0, 0, 0};    // back along the match loading bar
  autonCommands[2] = {-10, 45, 0, 0, 0, 0};   // turn so back faces net
  autonCommands[3] = {-20, 0, 0, 0, 0, 0};    // ram preload under net
  autonCommands[4] = {20, 0, 0, 0, 0, 0};     // drive back to ML bar
  autonCommands[5] = {10, -45, 0, 0, 2, 0};   // reallign with ML bar, open right wing
  autonCommands[6] = {45, 0, 0, 0, 2, 0};     // clear corner triball
  autonCommands[7] = {0, -45, 0, 0, 2, 0};    // turn to face gutter
  autonCommands[8] = {0, -21.8, 0, 0, 0, 0};  // close wing, turn to face lower middle triball (using trig!)
  autonCommands[9] = {110, 0, 0, 0, 0, 0};    // drive almost up to middle triball
  autonCommands[10] = {5, 0, 3, 0, 0, 0};     // drive to touch orb, raise arm
  autonCommands[11] = {5, 0, 2, 0, 0, 0};     // drive forward, capture orb
  autonCommands[12] = {0, 21.8, 0, 0, 3, 0};  // turn to face middle bar, extend left wing
  autonCommands[13] = {55, 0, 0, -70, 3, 0};  // push orbs over middle
  autonCommands[14] = {-90, 0, 0, 0, 0, 0};   // move back towards net
  autonCommands[15] = {0, 90, 0, 0, 0, 0};    // turn towards gutter
  autonCommands[16] = {150, 0, 0, 0, 0, 0};   // drive to mouth of gutter
  autonCommands[17] = {0, 90, 0, 0, 0, 0};    // turn to face gutter with back
  autonCommands[18] = {-105, 0, 3, 0, 0, 0};  // drive to push over corner orb / gutter orb, touch bar

  totalNumOfCommands = 19;
  autonCommands->resize(totalNumOfCommands);
}

void testAuton() {
  // autonCommands[ autonStep ] = {[]}
  //[lateralDistance(cm), rotationalDistance(degrees), flystickArmPos(1-5, 0 = no change), flywheelSpeed(%), wingsOut(bool), delay(seconds)]


  // simple test routine that drives in a square, raises and spins the flywheel for 5 seconds then lowers it and drives backwards
  autonCommands[0] = {0, 0, 0, 0, 0, 0};    // null padded start
  autonCommands[1] = {25, 0, 0, 0, 0, 0};   // fwd 25cm
  autonCommands[2] = {0, 90, 0, 0, 0, 0};   // right 90deg
  autonCommands[3] = {25, 0, 0, 20, 0, 1};  // fwd 25cm, 20% flywheel speed, pause 2 sec
  autonCommands[4] = {0, 90, 0, 0, 0, 0};   // right 90deg
  autonCommands[5] = {0, -90, 0, 0, 0, 0};  // fwd 25cm
  autonCommands[6] = {0, 90, 0, 0, 0, 0};   // right 90deg
  autonCommands[7] = {25, 0, 0, 0, 0, 0};   // fwd 25cm to starting pos
  autonCommands[8] = {0, 0, 3, 0, 0, 5};    // raise arm, 50% flywheel speed, pause 5 sec
  autonCommands[9] = {-15, 0, 1, 0, 0, 0};  // back 15cm, lower arm

  totalNumOfCommands = 10;
  autonCommands->resize(totalNumOfCommands);
}

#pragma endregion

// PID tunings
void setPIDTunings(int range) {
  switch (range) {
    case 0:
      lP = 0.65;
      lD = 0.25;
      lI = 0.35;
      lOutput = 1.9;

      rP = 0.80;
      rD = 0.55;
      rI = 1.70;
      rOutput = 2.0;
      break;
    case 1:
      lP = 0.70;
      lD = 0.30;
      lI = 0.45;
      lOutput = 1.0;

      rP = 1.25;
      rD = 0.40;
      rI = 0.30;
      rOutput = 2.9;
  }
}

#pragma region printingConfigs

void selectorPrinting() {  // innefficient but easier to read, hardcoding would be better
  currentPage = 1;

  isPrintingList[0] = false;  // debug - disabled
  isPrintingList[1] = true;   // Auton Selector - enabled
  isPrintingList[2] = false;  // Auton Route - disabled
  isPrintingList[3] = false;  // PID - disabled
  isPrintingList[4] = false;  // Drivetrain - disabled
  isPrintingList[5] = false;  // GPS - disabled
  isPrintingList[6] = false;  // Kinematic Controller - disabled
  isPrintingList[7] = false;  // Drive Tuning - disabled
  isPrintingList[8] = false;  // PID Tuning - disabled
}

void autonPrinting() {
  currentPage = 1;

  isPrintingList[0] = false;  // debug - disabled
  isPrintingList[1] = false;  // Auton Selector - disabled
  isPrintingList[2] = true;   // Auton Route - enabled
  isPrintingList[3] = true;   // PID - enabled
  isPrintingList[4] = false;  // Drivetrain - disabled
  isPrintingList[5] = true;   // GPS - enabled
  isPrintingList[6] = false;  // Kinematic Controller - disabled
  isPrintingList[7] = false;  // Drive Tuning - disabled
  isPrintingList[8] = false;  // PID Tuning - disabled
}

void userControlPrinting() {
  currentPage = 1;

  isPrintingList[0] = false;  // debug - disabled
  isPrintingList[1] = false;  // Auton Selector - disabled
  isPrintingList[2] = false;  // Auton Route - disabled
  isPrintingList[3] = false;  // PID - disabled
  isPrintingList[4] = true;   // Drivetrain - enabled
  isPrintingList[5] = true;   // GPS - enabled
  isPrintingList[6] = false;  // Kinematic Controller - disabled
  isPrintingList[7] = false;  // Drive Tuning - disabled
  isPrintingList[8] = false;  // PID Tuning - disabled
}

#pragma endregion  // printingConfigs

#pragma endregion


// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // //


#pragma region Pregame //code which executes before a game starts

void initialize() {
  selectorPrinting();

  WingPL.set_value(false);
  WingPR.set_value(false);

  mStartPosL = LDriveMidM.get_position();
  mStartPosR = RDriveMidM.get_position();

  FullDrive.set_brake_modes(E_MOTOR_BRAKE_COAST);

  Inertial.reset(true);
}

void disabled() {}

void competition_initialize() {  // auton selector
}


#pragma endregion  // Pregame


// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // //



void autonomous() {
  selectedRoute = 1;
  autonPrinting();

  int maxiumAutonTime = 15 * ticksPerSec;  // sets the default auton kill time to 15 seconds

  switch (selectedRoute) {
    case 1:
      skillsAuton();
      maxiumAutonTime = 120 * ticksPerSec;  // raises the kill time for skills auton to 60 seconds
      break;
    case 2:
      offenceAuton();
      break;
    case 3:
      defenceAuton();
      break;
    case 4:
      return;
      break;
    case 5:
      testAuton();
      maxiumAutonTime = 120 * ticksPerSec;
      break;
  }

  // autonStep = 13;
  desiredDist = 0;
  desiredHeading = 0;
  std::array<int, 6> nextCommand = {0, 0, 0, 0, 0, 0};

  while (globalTimer < maxiumAutonTime) {
    const float heading = std::fmod(Inertial.get_heading(), 360.0f);
    const float currHeading = (heading > 180) ? (heading - 360) : heading;

    lcdControl();  // allows the LCD screen to show multiple pages of diagnostics, press left/right arrows to change pages


    bool isCurrStepComplete = AutonPID(true);

    if ((autonStep + 1) == totalNumOfCommands) {  // temp. locks the program if auton route is complete
      currentPage = 1;

      FullDrive.move_velocity(0);

      if (globalTimer % 11) {
        MainControl.clear();
      }

      PrintToController("Out of bounds", 0, 0, 1, 1);

      globalTimer++;
      delay(tickDeltaTime);

      return;


    } else if ((MainControl.get_digital_new_press(DIGITAL_X)) && globalTimer > minStepChangeTimeStamp) {
      // || isCurrStepComplete  <- remove/place next to (DIGITAL_X) to exit/enter debug mode
      desiredDist += nextCommand.at(0) * degPerCM;
      desiredHeading += nextCommand.at(1);

      setPIDTunings((nextCommand.at(0) < 50 || nextCommand.at(1) < 90));  // calling the vector's subvalues multiple times is bad practice
      // i do not care atm (its mostly negligible since it isn't in the main loop)

      switch (nextCommand.at(4)) {
        case 0:  // no out all in
          WingPL.set_value(false);
          WingPR.set_value(false);
          break;
        case 1:  // all out no in
          WingPL.set_value(true);
          WingPR.set_value(true);
          break;
        case 2:  // 2 = right out left in
          WingPL.set_value(false);
          WingPR.set_value(true);
          break;
        case 3:  // 3 = left out right in
          WingPL.set_value(true);
          WingPR.set_value(false);
          break;
      }

      minStepChangeTimeStamp = globalTimer + GreaterOf(nextCommand.at(5), stepChangeCooldown);
      // current time plus step end delay = min time at which step can end

      autonStep++;
      nextCommand = ReadAutonStep(autonStep);
      delay(tickDeltaTime * minPrintingDelay);
    }

#pragma region Diagnostics

    // diagnostics section  |  [2] AutRoute - 1
    int startingPage = pageRangeFinder(2);

    PrintToController("Step: ", autonStep, 2, 0, startingPage);
    PrintToController("DesDist: ", desiredDist, 4, 1, startingPage);
    PrintToController("DesHead: ", desiredHeading, 3, 2, startingPage);

    PrintToController("OHeading: ", heading, 4, 0, startingPage + 1);
    PrintToController("RHeading: ", currHeading, 4, 1, startingPage + 1);
    PrintToController("ErrorH: ", proportionalErrorR, 3, 2, startingPage + 1);

    PrintToController("FWingsOut?: ", nextCommand.at(4), 1, 0, startingPage + 2);
    PrintToController("FArmPos: ", ((nextCommand.at(2) > 0) ? nextCommand.at(3) : armPos), 1, 1, startingPage + 2);
    PrintToController("FWheelSpd: ", nextCommand.at(3), 3, 2, startingPage + 2);

#pragma endregion  // Diagnostics

    globalTimer++;
    delay(tickDeltaTime);
  }
}



// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // //



void opcontrol() {
#pragma region Debugging

  // competition_initialize();
  // autonomous();

  // tunePID(true);

  // tuneDrive(true);

#pragma endregion

  userControlPrinting();

  while (true) {
    DrivingControl(true);
    WingsControl();
    RCIntakeControls();

    lcdControl();

    globalTimer++;
    delay(tickDeltaTime);
  }
}