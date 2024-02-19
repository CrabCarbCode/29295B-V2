#include "main.h"

#pragma region Variables

int selectorStage = 0;
int selectedRoute = 3;


const int stepChangeCooldown = ticksPerSec / 4;  // sets the minimum delay between auton steps

// initializing data variables (used to track details of each step)
int autonStep = 0;  // tracks which step of the auton the program is on
int flywheelSpeed = 0;
int minStepChangeTimeStamp;


bool ArmUp = false;  // check if arm start down
float prevErrorF = 0;

#pragma endregion



#pragma region Functions

int stuckTimeStamp = 0;
int avgMotorPosition = 0;

bool AutonPID(bool isPrinting);

bool ManageArm(bool isPrinting);

void tunePID(bool isPrinting);

#pragma endregion