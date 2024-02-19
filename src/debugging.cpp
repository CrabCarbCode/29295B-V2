#include "debugging.h"

#include "custPrinting.h"
#include "data-storage.h"
#include "main.h"
#include "robot-config.h"


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
      PrintToController("lOutput: ", lO, 2, 1, startPage + 1);
      PrintToController("rOutput: ", rO, 2, 2, startPage + 1);
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