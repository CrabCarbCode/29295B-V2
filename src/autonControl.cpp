#include "autonControl.h"

#include "data-storage.h"
#include "generalFuncs.h"
#include "robot-config.h"  //importing the motors and whatnot
#include "userControl.h"

// handles autonomous executions (no user input), quite high-level

#pragma region HelperFunctions

#pragma endregion



#pragma region MainFunctions

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

#pragma enregion



#pragma region AuxiliaryFunctions

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

#pragma region Debugging

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
      PrintToController("lOutput: ", lO, 2, 1, startPage + 1);
      PrintToController("rOutput: ", rO, 2, 2, startPage + 1);
    }


    globalTimer++;
    delay(tickDeltaTime);
  }
}

#pragma endregion