#pragma region BoringConfigStuff

#include "main.h"

#include <math.h>    //neccessary for functions like abs() and round()
#include <stdlib.h>  //neccessary for std::[commands]

#include <cmath>
#include <cstring>
#include <sstream>  //neccessary for... logic
#include <string>   //neccessary for... using strings :sob:

#include "custPrinting.h"
#include "data-storage.h"
#include "debugging.h"
#include "robot-config.h"  //importing the motors and whatnot

using namespace std;

// This is my code, and thus it is my god given right to use it as a diary. ignore the strange comments

#pragma endregion


// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // //


#pragma region GlobalVars

///// Control Variables //////


const int deadband = 3;  // if the controller sticks are depressed less than deadband%, input will be ignored (to combat controller drift)

const float autonDriveMult = 1.0;
// unused variable to increase / decrease speed of autonomous driving. just make a good drivetrain lol you'll be fine


int maxIntakeSpeed = 70;  // max intakeSpeed as a percent
int maxKickerSpeed = 60;
int armLevel = 1;
const int maxArmLevel = 2;

static int currentPage = 1;

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

    lateralPower = (proportionalErrorL + derivativeErrorL + integralErrorL) * lO;

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


#pragma region UserControlFunctions //handles all functions involving user input



#pragma region AutonomousFunctions //Functions for autonomous control

int selectorStage = 0;
int selectedRoute = 3;


const int stepChangeCooldown = ticksPerSec / 4;  // sets the minimum delay between auton steps

// initializing data variables (used to track details of each step)
int autonStep = 0;  // tracks which step of the auton the program is on
int flywheelSpeed = 0;
int minStepChangeTimeStamp;


bool ArmUp = false;  // check if arm start down
float prevErrorF = 0;

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

#pragma endregion  // end of AutonFunctions

#pragma endregion  // end of Bot controlling functions


// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // //



#pragma region debugFunctions



// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // //



#pragma region Pregame //code which executes before a game starts

void initialize() {
  selectorPrinting();

  WingPL.set_value(false);
  WingPR.set_value(false);

  FullDrive.set_brake_modes(E_MOTOR_BRAKE_COAST);

  KickerM.set_brake_mode(E_MOTOR_BRAKE_HOLD);

  Inertial.reset(true);
}

void disabled() {}

void competition_initialize() {  // auton selector
}


#pragma endregion  // Pregame


// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // //


#pragma region AutonomousExecution

void autonomous() {
  autonPrinting();

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
  }


  while (globalTimer <= maxAutonTime) {
    const float heading = std::fmod(Inertial.get_heading(), 360.0f);
    const float currHeading = (heading > 180) ? (heading - 360) : heading;

    lcdControl();

    bool isCurrStepComplete = AutonPID(true);

    if (autonStep >= totalNumOfCommands) {  // temp. locks the program if auton route is complete
      currentPage = 1;

      FullDrive.move_velocity(0);

      if (globalTimer % 11) { MainControl.clear(); }

      PrintToController("Out of bounds", 0, 0, 1, 1);

      globalTimer++;
      delay(tickDeltaTime);

      return;

    } else if ((MainControl.get_digital_new_press(DIGITAL_X)) && globalTimer > minStepChangeTimeStamp) {
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


#pragma endregion


// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // //



void opcontrol() {
#pragma region Debugging

  tunePID(true);

  // competition_initialize();
  // autonomous();

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

    lcdControl();

    globalTimer++;
    delay(tickDeltaTime);
  }
}