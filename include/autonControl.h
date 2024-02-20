#include "main.h"

#pragma region Variables

int selectorStage = 0;
int selectedRoute = 3;

// initializing data variables (used to track details of each step)
int autonStep = 0;  // tracks which step of the auton the program is on
int minStepChangeTimeStamp;


bool ArmUp = false;  // check if arm start down
float prevErrorF = 0;

#pragma endregion



#pragma region Functions

int stuckTimeStamp = 0;
int avgMotorPosition = 0;

float adjustFactor = 0.05;  // the increment by which PID variables change during manual tuning
bool isTuningTurns = true;

bool AutonPID(bool isPrinting);

bool ManageArm(bool isPrinting);

void tunePID(bool isPrinting);

#pragma endregion