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


const int deadband = 3;  // if the controller sticks are depressed less than deadband%, input will be ignored (to combat controller drift)

const float autonDriveMult = 1.0;
// unused variable to increase / decrease speed of autonomous driving. just make a good drivetrain lol you'll be fine

const float Pi = 3.14159265358;
const float e = 2.71828182845;

int globalTimer = 0;
const int ticksPerSec = 50;  // the number of 'ticks' in one second
const int tickDeltaTime = 1000 / ticksPerSec;
int minPrintingDelay = (ticksPerSec / tickDeltaTime) + 0.5;  // ticksPerSec / tickDeltaTime

const float degPerCM = (360 / (4.1875 * Pi * 2.54)) * (84.0f / 36.0);  // # of degrees per centimeter = 360 / (2Pir" * 2.54cm/") * gear ratio

int maxIntakeSpeed = 100;  // max intakeSpeed as a percent
int maxKickerSpeed = 75;
int armLevel = 1;
int maxArmLevel = 2;

int currentPage = 1;

int selectedRoute = 0;
int autonStep = 0;
int minStepChangeTimeStamp;


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


#pragma region AutonControl //the code behind the autonomous Proportional Integral Derivative controller

#pragma region Variables // holds all variables required for the PID controller

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

#pragma region AuxiliaryFunctions

int prevErrorF = 0;

bool ManageArm(bool isPrinting) {
  const float armDeadband = 0.25;

  const float fP = 1.0;
  const float fD = 1.0;
  const float fO = 1.0;

  float desArmPos;
  switch (armLevel) {
    case 0:
      break;
    case 1:
      desArmPos = 0;
      break;
    case 2:
      desArmPos = 698;
      break;
  }

  const float currArmPos = ArmRot.get_position() / 100;

  const float currErrorF = fP * (currArmPos - desArmPos);
  const float currErrorD = fD * (currErrorF - prevErrorF) / 2;

  if (isPrinting && isPrintingList[0]) {
    PrintToController("Position: ", armLevel, 1, 0, 1);
    PrintToController("Rotation: ", currArmPos, 5, 1, 1);
    PrintToController("Error: ", currErrorF, 5, 2, 1);
  }

  prevErrorF = currErrorF;

  if (fabs(currErrorF) > armDeadband) {  // if target has not been hit
    LiftM.move_velocity(fO * (currErrorF + currErrorD));
    return false;
  } else {  // if target has been hit
    LiftM.set_brake_mode(E_MOTOR_BRAKE_HOLD);
    LiftM.move_velocity(0);
    return true;
  }
}


#pragma endregion



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

    lateralPower = (proportionalErrorL + derivativeErrorL + integralErrorL) * lO;

    previousErrorL = proportionalErrorL;

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

    rotationalPower = (proportionalErrorR + derivativeErrorR + integralErrorR) * rO;

    previousErrorR = proportionalErrorR;

    ///////////////////////////////////////
    //////        ending math        //////
    ///////////////////////////////////////

    LDrive(autonDriveMult * (lateralPower + rotationalPower));
    RDrive(autonDriveMult * (lateralPower - rotationalPower));

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


#pragma region tuning

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

  lO = 1.0;

  rP = 0.5;
  rD = 0.0;
  rI = 0.0;

  rO = 1.0;

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
      lO += adjustFactor;
    }
    if (MainControl.get_digital_new_press(DIGITAL_L1)) {  // changes output power of rotational PID
      rO += adjustFactor;
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
      PrintToController("lO: ", lO, 2, 1, startPage + 1);
      PrintToController("rO: ", rO, 2, 2, startPage + 1);
    }


    globalTimer++;
    delay(tickDeltaTime);
  }
}


#pragma endregion  // tuning


#pragma endregion  // end of PID block


// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // //


#pragma region UserControl

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


#pragma region HelperFunctions


// i have no idea what im doing
float AccelSmoothingFunc(int time) {  // returns a multiplier 0 to 1 based on time
  float x = time / 100;               // converting the input from percentage to a decimal btween 0-1


  const float multiplier =
      (0.5 / (ACurveExtremity * sqrt(2 * Pi))) * powf(e, (-0.5 * pow(((AMinAmount * x - AMinAmount * peakPos) / ACurveExtremity), 2)));
  return time >= (100) ? multiplier : 1;
}  // function graphed here: [https://www.desmos.com/calculator/y0fwlh6j47]



// i now have some idea what im doing
float StickSmoothingFunc(float stickVal) {
  float curveExponent = (abs(stickVal) - 100) / (10 * ACurveExtremity);
  float linearExponent = (-1 / (10 * linearHarshness));


  return (stickVal * (powf(e, linearExponent) + ((1 - powf(e, linearExponent)) * powf(e, curveExponent))));
}  // function graphed here: [https://www.desmos.com/calculator/ti4hn57sa7]


#pragma endregion



#pragma region MainFunctions


void DrivingControl(bool isPrinting) {  // responsible for user control of the drivetrainint gift


  if (MainControl.get_digital_new_press(DIGITAL_A)) {
    reverseDriveMult = (reverseDriveMult == 1) ? -1 : 1;
  }


  // taking the position of the sticks and appplying gradient diffusion to them. Check the StickSmoothingFunc graph for details
  // X stick covers fwd/back, Y stick covers turning


  float XStickPercent = StickSmoothingFunc(MainControl.get_analog(ANALOG_LEFT_Y) / 1.27 * reverseDriveMult);  // w on graph
  float YStickPercent = StickSmoothingFunc(MainControl.get_analog(ANALOG_RIGHT_X) / 1.27);                    // s on graph


  const float ptsPerTick = 4;


  // filter out stick drift / nonpressed sticks. saves resources by skipping calculations when not driving
  if ((abs(XStickPercent) + abs(YStickPercent)) >= deadband) {
    int fullStopThreshold = 150;


    // inreasing or decreasing the acceleration functions' timer
    LAccelTime += ((LAccelTime <= 100 && XStickPercent > deadband) || LAccelTime < 0) ? ptsPerTick : -ptsPerTick;  // Y(x) on graph
    RAccelTime += ((RAccelTime <= 100 && YStickPercent > deadband) || RAccelTime < 0) ? 1 : -1;                    // X(x) on graph



    // applying the acceleratory curve to the stick inputs, multiplies the stick values by the output of the accel smoothing function


    int lateralOutput = AccelSmoothingFunc(LAccelTime) * XStickPercent;


    // lowers the rotational output based on the lateral output, to prevent turning from overpowering fwd motion at high speeds
    float rotationalMult = ((-0.001 * powf(lateralOutput, 2)) + (-0.25 * lateralOutput) + 70) / 100;
    // graphed here: [https://www.desmos.com/calculator/03mizqcj4f]


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


    // its unfortunate that I have to write the printing setup twice, but this way we avoid calculations when not driving


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


#pragma endregion



#pragma region AuxiliaryFunctions


// handles user control of intake
void RCIntakeControls() { IntakeM.move_velocity(maxIntakeSpeed * 6 * (MainControl.get_digital(DIGITAL_R2) - MainControl.get_digital(DIGITAL_L2))); }


void KickerControl() {
  bool kickerButton = (SideControl.is_connected()) ? SideControl.get_digital(DIGITAL_A) : MainControl.get_digital(DIGITAL_A);
  KickerM.move_velocity(maxKickerSpeed * kickerButton);  // if not pressed, 0 * x, if pressed, 1 * x
}


void ControlArm() {
  // not a fan of this solution for switching which controller is in charge of the arm, but I can't think of a generalized one right now


  bool UpButton = (SideControl.is_connected()) ? SideControl.get_digital_new_press(DIGITAL_UP) : MainControl.get_digital_new_press(DIGITAL_UP);
  bool DownButton = (SideControl.is_connected()) ? SideControl.get_digital_new_press(DIGITAL_DOWN) : MainControl.get_digital_new_press(DIGITAL_DOWN);


  if (UpButton && armLevel < maxArmLevel) {
    LiftM.set_brake_mode(E_MOTOR_BRAKE_COAST);
    armLevel++;
  }
  if (DownButton && armLevel > 1) {
    LiftM.set_brake_mode(E_MOTOR_BRAKE_COAST);
    armLevel--;
  }
}

bool LWingLockedOut = false;
bool RWingLockedOut = false;

void WingsControl() {  // controls... wings


  if (MainControl.get_digital_new_press(DIGITAL_L1)) {
    LWingLockedOut = !LWingLockedOut;
    WingPL.set_value(LWingLockedOut);
  }


  if (MainControl.get_digital_new_press(DIGITAL_R1)) {
    RWingLockedOut = !RWingLockedOut;
    WingPR.set_value(RWingLockedOut);
  }
}


#pragma endregion



#pragma region Tuning


void tuneDrive(bool isPrinting) {  // allows for user driving, with real time control over drive coefficients
  // default settings for acceleratory / stick curves


  isPrintingList[7] = true;  // Drive Tuning - disabled


  ACurveExtremity = 0.19948;  // sigma
  AMinAmount = 0.235;         // kappa


  linearHarshness = 0.2;  // g on graph
  SCurveExtremity = 4.7;  // h on graph


  float changeAmount = 1;


  while (true) {
    lcdControl();


    if (MainControl.get_digital_new_press(DIGITAL_X)) {
      ACurveExtremity += changeAmount / 100000;
    }
    if (MainControl.get_digital_new_press(DIGITAL_A)) {
      AMinAmount += changeAmount / 1000;
    }
    if (MainControl.get_digital_new_press(DIGITAL_B)) {
      linearHarshness += changeAmount / 20;
    }
    if (MainControl.get_digital_new_press(DIGITAL_Y)) {
      SCurveExtremity += changeAmount / 10;
    }


    if (MainControl.get_digital_new_press(DIGITAL_UP)) {
      changeAmount++;
    }
    if (MainControl.get_digital_new_press(DIGITAL_DOWN)) {
      changeAmount--;
    }
    if (MainControl.get_digital_new_press(DIGITAL_L1)) {
      changeAmount *= -1;
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

#pragma endregion


// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // //


#pragma region dataStorage // should be ported to individual header files at some point

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

#pragma region autonRoutes


int debugRoute(autonCommand currCommandList[]) {
  // dist /rotation: in cm / degrees (relative to current position)
  // wings : 0 = niether, 1 = both, 2 = left, 3 = right, relative to intake as back
  // intake / kickers: in % speed, delay is in seconds
  // arm position: higher number = more up


  const int numOfCommands = 8;


  currCommandList[1].lateralDist = 25;  // drives fwd 25 cm


  currCommandList[2].rotationalDist = 90;  // turn 90deg right


  currCommandList[3].lateralDist = 25;  // drive fwd with both wings out
  currCommandList[3].wingPattern = 1;   //


  currCommandList[4].lateralDist = 25;     // retract wings, try a curved drive path right/fwd 25
  currCommandList[4].rotationalDist = 90;  //
  currCommandList[4].wingPattern = 0;      //


  currCommandList[5].lateralDist = 25;   // raise the arm, run the intake and drive fwd 25cm
  currCommandList[5].armPosition = 2;    //
  currCommandList[5].intakeSpeed = 100;  //


  currCommandList[6].rotationalDist = 90;  // turn 90deg right,
  currCommandList[6].kickerSpeed = 75;     // shut off the intake (automatic)
  currCommandList[6].totalStepDelay = 5;   //


  currCommandList[7].armPosition = 1;       // turn 90deg left, lower arm
  currCommandList[7].rotationalDist = -90;  //


  currCommandList[8].lateralDist = -25;  // drive bwd 25cm, to starting area


  currCommandList[9].rotationalDist = 720;  // try to go out of bounds, spin a lot


  return numOfCommands;
}


// Defensive routes

int safeDefenceRoute(autonCommand currCommandList[]) {
  // dist /rotation: in cm / degrees (relative to current position)
  // wings : 0 = niether, 1 = both, 2 = left, 3 = right, relative to intake as back
  // intake / kickers: in % speed, delay is in seconds
  // arm position: higher number = more up


  const int numOfCommands = 18;


  currCommandList[1].lateralDist = 78;  // move fwd along ML bar


  currCommandList[2].rotationalDist = 45;  // turn to face net


  currCommandList[3].lateralDist = 8;     // drive up to net
  currCommandList[3].intakeSpeed = -100;  // outtake


  currCommandList[4].lateralDist = -14;  // move bwd towards ML bar


  currCommandList[5].rotationalDist = -45;  // turn to be parallel with ML bar
  currCommandList[5].wingPattern = 3;       // open right wing


  currCommandList[6].lateralDist = -43;  // move bwd along ML bar, clearing corner orb
  currCommandList[6].wingPattern = 3;    // keep right wing open


  currCommandList[7].rotationalDist = -45;  // turn to !face gutter, pushing orb (close wing)


  currCommandList[8].rotationalDist = 114;  // turn to face lower middle orb


  currCommandList[9].lateralDist = 124;  // move almost to lower middle orb


  currCommandList[10].lateralDist = 5;    // move fwd slightly
  currCommandList[10].intakeSpeed = 100;  // intake orb


  currCommandList[11].rotationalDist = -114;  // turn to !face middle barrier


  currCommandList[12].lateralDist = -48;  // move bwd to push orb over middle
  currCommandList[12].wingPattern = 2;    // extend left wing


  currCommandList[13].lateralDist = 18;  // move fwd to be centered on small gutter barrier


  currCommandList[14].rotationalDist = -90;  // turn to face small gutter barrier


  currCommandList[15].lateralDist = 135;  // move into gutter over bar


  currCommandList[16].rotationalDist = -90;  // turn to face middle barrier


  currCommandList[17].lateralDist = 20;    // move fwd to push orbs over
  currCommandList[17].intakeSpeed = -100;  // outtake


  currCommandList[18].totalStepDelay = 5;  // delay


  return numOfCommands;
}


int rushDefenceRoute(autonCommand currCommandList[]) {
  // dist /rotation: in cm / degrees (relative to current position)
  // wings : 0 = niether, 1 = both, 2 = left, 3 = right, relative to intake as back
  // intake / kickers: in % speed, delay is in seconds
  // arm position: higher number = more up


  const int numOfCommands = 18;



  return numOfCommands;
}


// Offensive routes



int safeOffenceRoute(autonCommand currCommandList[]) {
  // dist /rotation: in cm / degrees (relative to current position)
  // wings : 0 = niether, 1 = both, 2 = left, 3 = right, relative to intake as back
  // intake / kickers: in % speed, delay is in seconds
  // arm position: higher number = more up


  const int numOfCommands = 20;


  return numOfCommands;
}


int rushOffenceRoute(autonCommand currCommandList[]) {
  // dist /rotation: in cm / degrees (relative to current position)
  // wings : 0 = niether, 1 = both, 2 = left, 3 = right, relative to intake as back
  // intake / kickers: in % speed, delay is in seconds
  // arm position: higher number = more up


  const int numOfCommands = 18;


  currCommandList[1].intakeSpeed = -100;  // outtake preload


  currCommandList[2].rotationalDist = -62.8;  // turn to face lower middle orb


  currCommandList[3].lateralDist = 134.5;  // move up to orb
  currCommandList[3].intakeSpeed = 100;    // intake


  currCommandList[4].rotationalDist = 107.8;  // turn to face net


  currCommandList[5].intakeSpeed = -100;  // outtake orb


  currCommandList[6].rotationalDist = -167.5;  // turn to face upper middle orb


  currCommandList[7].lateralDist = 40.5;  // move up to orb
  currCommandList[7].intakeSpeed = 100;   // intake


  currCommandList[8].rotationalDist = 167.5;  // turn to face net


  currCommandList[9].lateralDist = 75;  // move towards net


  currCommandList[10].lateralDist = 10;    // move to touch net
  currCommandList[10].intakeSpeed = -100;  // outtake


  currCommandList[11].lateralDist = -10;  // back up


  currCommandList[11].rotationalDist = 148.2;  // turn to face corner orb


  currCommandList[12].lateralDist = 96;   // move up to orb
  currCommandList[12].intakeSpeed = 100;  // intake


  currCommandList[13].rotationalDist = -101.9;  // turn to face original position


  currCommandList[14].lateralDist = 126;  // move almost up to ML bar


  currCommandList[15].rotationalDist = -90;  // turn to face net
  currCommandList[16].intakeSpeed = -100;    // outtake orb


  currCommandList[17].rotationalDist = -180;  // turn to !face along ML bar


  currCommandList[18].lateralDist = -41;  // move along ML bar
  currCommandList[18].wingPattern = 3;    // extend right wing


  currCommandList[19].rotationalDist = -32;  // turn to directly !face net
  currCommandList[19].wingPattern = 3;       // keep right wing extended


  currCommandList[20].lateralDist = -45;  // push orbs under net
  currCommandList[20].wingPattern = 2;    // extend left wing


  return numOfCommands;
}



// Skills routes

int fullSkillsRoute(autonCommand currCommandList[]) {
  // dist /rotation: in cm / degrees (relative to current position)
  // wings : 0 = niether, 1 = both, 2 = left, 3 = right, relative to intake as back
  // intake / kickers: in % speed, delay is in seconds
  // arm position: higher number = more up


  const int numOfCommands = 18;


  currCommandList[1].lateralDist = -55;


  currCommandList[2].rotationalDist = 45;


  currCommandList[3].lateralDist = -30;


  currCommandList[4].lateralDist = 37;


  currCommandList[5].lateralDist = 5;
  currCommandList[5].rotationalDist = 90;


  currCommandList[6].lateralDist = -12;
  currCommandList[6].rotationalDist = -14;
  currCommandList[6].armPosition = 3;


  currCommandList[7].kickerSpeed = 90;
  currCommandList[7].totalStepDelay = 30;


  currCommandList[8].lateralDist = 14;
  currCommandList[8].armPosition = 1;


  currCommandList[9].rotationalDist = -130;


  currCommandList[10].lateralDist = 78;


  currCommandList[11].lateralDist = 16;
  currCommandList[11].rotationalDist = -38;


  currCommandList[12].lateralDist = 105;


  currCommandList[13].lateralDist = 61;


  currCommandList[14].rotationalDist = -45;
  currCommandList[14].wingPattern = 2;


  currCommandList[15].lateralDist = 69;
  currCommandList[15].wingPattern = 2;


  currCommandList[16].lateralDist = 10;
  currCommandList[16].rotationalDist = -45;


  currCommandList[17].lateralDist = 20;


  currCommandList[18].lateralDist = -30;


  currCommandList[19].lateralDist = 30;


  currCommandList[20].lateralDist = -26;


  currCommandList[21].rotationalDist = 94;


  currCommandList[22].lateralDist = -110;


  currCommandList[23].rotationalDist = -46;


  currCommandList[23].wingPattern = 1;


  currCommandList[24].lateralDist = 85;
  currCommandList[24].wingPattern = 1;


  currCommandList[25].lateralDist = -30;
  currCommandList[25].wingPattern = 2;


  currCommandList[26].lateralDist = 30;
  currCommandList[26].wingPattern = 1;


  currCommandList[27].lateralDist = -75;


  currCommandList[28].rotationalDist = -45;


  currCommandList[29].lateralDist = 50;
  currCommandList[29].wingPattern = 3;


  currCommandList[30].rotationalDist = 45;
  currCommandList[30].wingPattern = 3;


  currCommandList[31].lateralDist = 85;
  currCommandList[31].wingPattern = 1;


  currCommandList[32].lateralDist = -30;
  currCommandList[32].wingPattern = 2;


  currCommandList[33].lateralDist = 30;
  currCommandList[33].wingPattern = 1;


  currCommandList[34].lateralDist = -65;


  currCommandList[35].rotationalDist = -115;


  currCommandList[36].lateralDist = 45;


  currCommandList[37].rotationalDist = -50;


  currCommandList[38].lateralDist = -45;
  currCommandList[38].wingPattern = 1;


  currCommandList[39].rotationalDist = -45;


  currCommandList[40].lateralDist = 110;


  currCommandList[41].rotationalDist = -90;


  currCommandList[42].lateralDist = 110;


  currCommandList[43].rotationalDist = -90;


  currCommandList[44].lateralDist = 100;


  currCommandList[45].rotationalDist = -90;


  currCommandList[46].lateralDist = 30;


  currCommandList[47].lateralDist = -30;


  currCommandList[48].lateralDist = 30;


  currCommandList[49].lateralDist = -60;


  return numOfCommands;
}


int driverSkillsRoute(autonCommand currCommandList[]) {
  // dist /rotation: in cm / degrees (relative to current position)
  // wings : 0 = niether, 1 = both, 2 = left, 3 = right, relative to intake as back
  // intake / kickers: in % speed, delay is in seconds
  // arm position: higher number = more up


  const int numOfCommands = 18;


  return numOfCommands;
}


#pragma endregion


// PID tunings
void SetPIDTunings(int range) {
  switch (range) {
    case 0:  // close  ( < 20cm / > 22.5deg)
      lP = 0.65;
      lD = 0.25;
      lI = 0.35;
      lO = 1.9;


      integralBoundL = 3;


      rP = 0.80;
      rD = 0.55;
      rI = 1.70;
      rO = 2.0;


      integralBoundR = 0.75;
      break;
    case 1:  // medium (20cm < 60cm / 22.5deg < 90deg)


    case 2:  // long ( > 60cm / > 90deg)
      lP = 0.70;
      lD = 0.30;
      lI = 0.45;
      lO = 1.0;


      integralBoundL = 8;


      rP = 1.25;
      rD = 0.40;
      rI = 0.30;
      rO = 2.9;


      integralBoundR = 4;
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

  FullDrive.set_brake_modes(E_MOTOR_BRAKE_COAST);

  Inertial.reset(true);
}

void disabled() {}

void competition_initialize() {  // auton selector
}


#pragma endregion  // Pregame


// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // //



void autonomous() {
  autonPrinting();
  selectedRoute = 7;


  autonCommand commandList[50];


  int currAutonStep = 0;
  int maxAutonTime = 15 * ticksPerSec;  // default time alloted for game autonomous routines
  int totalNumOfCommands = 1;


  // selectedRoute = x;


  switch (selectedRoute) {
    case 0:  // no auton selected, pure driving
      maxAutonTime = 0 * ticksPerSec;
      break;
    case 1:
      totalNumOfCommands += safeDefenceRoute(commandList);
      break;
    case 2:
      totalNumOfCommands += rushDefenceRoute(commandList);
      break;
    case 3:
      totalNumOfCommands += safeOffenceRoute(commandList);
      break;
    case 4:
      totalNumOfCommands += safeOffenceRoute(commandList);
      break;
    case 5:
      totalNumOfCommands += fullSkillsRoute(commandList);
      maxAutonTime = 60 * ticksPerSec;
      break;
    case 6:
      totalNumOfCommands += driverSkillsRoute(commandList);
      break;
    case 7:
      totalNumOfCommands += debugRoute(commandList);
      break;
  }



  while (globalTimer <= maxAutonTime) {
    const float heading = std::fmod(Inertial.get_heading(), 360.0f);
    const float currHeading = (heading > 180) ? (heading - 360) : heading;


    lcdControl();


    bool isCurrStepComplete = AutonPID(true);


    if (autonStep >= totalNumOfCommands) {  // temp. locks the program if auton route is complete
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
      //(MainControl.get_digital_new_press(DIGITAL_X))
      desiredDist += commandList[currAutonStep].lateralDist * degPerCM;
      desiredHeading += commandList[currAutonStep].rotationalDist * degPerCM;


      if (abs(commandList[currAutonStep].lateralDist) > 60 || abs(commandList[currAutonStep].rotationalDist) > 90) {
        SetPIDTunings(2);
      } else if (abs(commandList[currAutonStep].lateralDist) > 20 || abs(commandList[currAutonStep].rotationalDist) > 22.5) {
        SetPIDTunings(1);
      } else {
        SetPIDTunings(0);
      }


      switch (commandList[currAutonStep].wingPattern) {
        case 0:  // no out all in
          WingPL.set_value(false);
          WingPR.set_value(false);
          break;
        case 1:  // all out no in
          WingPL.set_value(true);
          WingPR.set_value(true);
          break;
        case 2:  // 2 = right out left in
          WingPL.set_value(true);
          WingPR.set_value(false);
          break;
        case 3:  // 3 = left out right in
          WingPL.set_value(false);
          WingPR.set_value(true);
          break;
      }


      armLevel = (commandList[currAutonStep].armPosition != -1) ? commandList[currAutonStep].armPosition : armLevel;
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


    PrintToController("FWingsOut?: ", commandList[currAutonStep].wingPattern, 1, 0, startingPage + 2);
    PrintToController("FArmPos: ", commandList[currAutonStep].armPosition, 1, 1, startingPage + 2);
    PrintToController("KickerSpeed: ", commandList[currAutonStep].kickerSpeed, 3, 2, startingPage + 2);


#pragma endregion  // Diagnostics


    globalTimer++;
    delay(tickDeltaTime);
  }
}



// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // //



void opcontrol() {
#pragma region Debugging


  // tunePID(true);


  competition_initialize();
  autonomous();


  // tuneDrive(true);


#pragma endregion


  userControlPrinting();
  isPrintingList[0] = true;


  while (true) {
    DrivingControl(true);
    WingsControl();
    RCIntakeControls();
    ControlArm();
    ManageArm(true);


    if (MainControl.get_digital(DIGITAL_A)) {
      KickerM.move_velocity(2 * maxKickerSpeed);
    } else {
      KickerM.move_velocity(0);
    }


    // switch activate second radio if controller gets disconnected
    if (!MainControl.is_connected()) {
    }


    lcdControl();


    globalTimer++;
    delay(tickDeltaTime);
  }
}
