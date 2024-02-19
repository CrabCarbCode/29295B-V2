#include "main.h"

// This is my code, and thus it is my god given right to use it as a diary. ignore the strange comments


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
