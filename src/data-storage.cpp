#include "data-storage.h"

#include <math.h>    //neccessary for functions like abs() and round()
#include <stdlib.h>  //neccessary for std::[commands]

#include <cmath>
#include <cstring>
#include <sstream>  //neccessary for... logic
#include <string>   //neccessary for... using strings :sob:

#include "custPrinting.h"


#pragma region autonRoutes

int debugRoute(autonCommand currCommandList[]) {
  // dist /rotation: in cm / degrees (relative to current position)
  // wings : 0 = niether, 1 = both, 2 = left, 3 = right, relative to intake as back
  // intake / kickers: in % speed, delay is in seconds
  // arm position: higher number = more up

  const int totalNumOfCommands = 8;

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

  return totalNumOfCommands;
}


// Defensive routes


int safeDefenceRoute(autonCommand currCommandList[]) {
  // dist /rotation: in cm / degrees (relative to current position)
  // wings : 0 = niether, 1 = both, 2 = left, 3 = right, relative to intake as back
  // intake / kickers: in % speed, delay is in seconds
  // arm position: higher number = more up

  const int totalNumOfCommands = 18;

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

  return totalNumOfCommands;
}

int rushDefenceRoute(autonCommand currCommandList[]) {
  // dist /rotation: in cm / degrees (relative to current position)
  // wings : 0 = niether, 1 = both, 2 = left, 3 = right, relative to intake as back
  // intake / kickers: in % speed, delay is in seconds
  // arm position: higher number = more up

  const int totalNumOfCommands = 18;


  return totalNumOfCommands;
}


// Offensive routes


int safeOffenceRoute(autonCommand currCommandList[]) {
  // dist /rotation: in cm / degrees (relative to current position)
  // wings : 0 = niether, 1 = both, 2 = left, 3 = right, relative to intake as back
  // intake / kickers: in % speed, delay is in seconds
  // arm position: higher number = more up

  const int totalNumOfCommands = 20;
}

int rushOffenceRoute(autonCommand currCommandList[]) {
  // dist /rotation: in cm / degrees (relative to current position)
  // wings : 0 = niether, 1 = both, 2 = left, 3 = right, relative to intake as back
  // intake / kickers: in % speed, delay is in seconds
  // arm position: higher number = more up

  const int totalNumOfCommands = 18;

  const int totalNumOfCommands = 18;

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

  return totalNumOfCommands;
}


// Skills routes


int fullSkillsRoute(autonCommand currCommandList[]) {
  // dist /rotation: in cm / degrees (relative to current position)
  // wings : 0 = niether, 1 = both, 2 = left, 3 = right, relative to intake as back
  // intake / kickers: in % speed, delay is in seconds
  // arm position: higher number = more up

  const int totalNumOfCommands = 18;
}

int driverSkillsRoutes(autonCommand currCommandList[]) {
  // dist /rotation: in cm / degrees (relative to current position)
  // wings : 0 = niether, 1 = both, 2 = left, 3 = right, relative to intake as back
  // intake / kickers: in % speed, delay is in seconds
  // arm position: higher number = more up

  const int totalNumOfCommands = 18;
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

#pragma endregion
