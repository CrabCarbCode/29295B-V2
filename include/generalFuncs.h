#include "main.h"
#include "robot-config.h"


#pragma region HelperFunctions

const char *toChar(std::string string);

int toInt(float val);

int timeSincePoint(int checkedTime);

template <typename T>
const float GreaterOf(T num1, T num2);

const bool IsWithinRange(float num, float lowerBound, float upperBound);

bool LDrive(float desPowerPercent);
bool RDrive(float desPowerPercent);

#pragma endregion

#pragma region Printing

int currentPage = 1;
int globalTimer = 0;

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

void lcdControl();

int pageRangeFinder(int index);

void PrintToController(std::string prefix, double data, int numOfDigits, int row, int page);

template <typename T, size_t N>
void PrintToController(std::string prefix, const std::array<T, N> &data, int numOfDigits, int row, int page);

#pragma endregion