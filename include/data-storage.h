#include <math.h>    //neccessary for functions like abs() and round()
#include <stdlib.h>  //neccessary for std::[commands]

#include <cmath>
#include <cstring>
#include <sstream>  //neccessary for... logic
#include <string>   //neccessary for... using strings :sob:

#pragma region autonRoutes
template <typename T, size_t N>
int totalNumOfCommands;

void skillsAuton(array < float, ) {
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