#include "userControl.h"

// This file handles the user's inputs on the program, from driving to tuning to control of other mechanisms

#pragma region Variables


// variables which control the shape/range of the acceleratory curve
float ACurveExtremity = 0.1996;  // sigma
float peakPos = 1;               // mu
float AMinAmount = 0.24;         // kappa

// variables which control the shape of the stick curve
float linearHarshness = 0.6;  // g on graph
float SCurveExtremity = 5.3;  // h on graph


#pragma endregion



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

  if (MainControl.get_digital_new_press(DIGITAL_A)) { reverseDriveMult = (reverseDriveMult == 1) ? -1 : 1; }

  // taking the position of the sticks and appplying gradient diffusion to them. Check the StickSmoothingFunc graph for details
  // X stick covers fwd/back, Y stick covers turning

  float XStickPercent = StickSmoothingFunc(MainControl.get_analog(E_CONTROLLER_ANALOG_LEFT_Y) / 1.27 * reverseDriveMult);  // w on graph
  float YStickPercent = StickSmoothingFunc(MainControl.get_analog(E_CONTROLLER_ANALOG_RIGHT_X) / 1.27);                    // s on graph

  const float ptsPerTick = 4;

  // filter out stick drift / nonpressed sticks. saves resources by skipping calculations when not driving
  if ((abs(XStickPercent) + abs(YStickPercent)) >= deadband) {
    int fullStopThreshold = 150;

    // inreasing or decreasing the acceleration functions' timer
    LAccelTime += ((LAccelTime <= 100 && XStickPercent > deadband) || LAccelTime < 0) ? ptsPerTick : -ptsPerTick;  // Y(x) on graph
    RAccelTime += ((RAccelTime <= 100 && YStickPercent > deadband) || RAccelTime < 0) ? 1 : -1;                    // X(x) on graph


    // applying the acceleratory curve to the stick inputs, multiplies the stick values by the output of the accel smoothing function

    int lateralOutput = AccelSmoothingFunc(LAccelTime) * XStickPercent;

    float rotationalMult = ((-0.001 * powf(lateralOutput, 2)) + (-0.25 * lateralOutput) + 80) / 100;
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
      if (!isPrintingList[4]) { isPrintingList[4] = true; }

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
      if (!isPrintingList[4]) { isPrintingList[4] = true; }

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
void RCIntakeControls() { IntakeM.move_velocity(maxIntakeSpeed * 6 * (MainControl.get_digital(DIGITAL_L1) - MainControl.get_digital(DIGITAL_R1))); }

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

#pragma endregion



#pragma region Tuning

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

    if (MainControl.get_digital_new_press(DIGITAL_X)) { ACurveExtremity += adjustFactor / 100000; }
    if (MainControl.get_digital_new_press(DIGITAL_A)) { AMinAmount += adjustFactor / 1000; }
    if (MainControl.get_digital_new_press(DIGITAL_B)) { linearHarshness += adjustFactor / 20; }
    if (MainControl.get_digital_new_press(DIGITAL_Y)) { SCurveExtremity += adjustFactor / 10; }

    if (MainControl.get_digital_new_press(DIGITAL_UP)) { adjustFactor++; }
    if (MainControl.get_digital_new_press(DIGITAL_DOWN)) { adjustFactor--; }
    if (MainControl.get_digital_new_press(DIGITAL_L1)) { adjustFactor *= -1; }

    DrivingControl(true);


    if (isPrinting) {  // [7] Drive Tune - 1
      if (!isPrintingList[7]) { isPrintingList[7] = true; }

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