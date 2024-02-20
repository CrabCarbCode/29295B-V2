#include "generalFuncs.h"

#include "autonControl.h"
#include "data-storage.h"
#include "robot-config.h"  //importing the motors and whatnot
#include "userControl.h"

// stores the helper functions used accross all levels of the program, very low-level

#pragma region HelperFunctions

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



#pragma region Printing

void lcdControl() {
  if (globalTimer % 11 == 0) {  // refresh the screen every 11 ticks because 11 is a good number :)
    MainControl.clear();
  }

  if (MainControl.get_digital_new_press(DIGITAL_LEFT) && currentPage > 0) {
    currentPage--;
  }
  if (MainControl.get_digital_new_press(DIGITAL_RIGHT)) {
    currentPage++;
  }
}

int pageRangeFinder(int index) {                 // calculates which page(s) a
  int startingPage = isPrintingList[0] ? 1 : 0;  // given function should print to
  // start on page 1
  for (int j = 0; j < index; ++j) {
    startingPage += isPrintingList[j] ? pagesPerPrint[j] : 0;
  }

  return startingPage;
}

void PrintToController(std::string prefix, double data, int numOfDigits, int row, int page) {  // handles single numbers
  if (currentPage == page && (globalTimer % 9 == (row * 3))) {
    std::string output = prefix + std::to_string(data).substr(0, numOfDigits + 1);
    // takes the first n digits of the number, adds it to output as string

    MainControl.print(row, 0, output.c_str(), 0);
  }
}


template <typename T, size_t N>
void PrintToController(std::string prefix, const std::array<T, N> &data, int numOfDigits, int row, int page) {  // handles multiple numbers
  if (currentPage == page && (globalTimer % 9 == (row * 3))) {
    std::string output = prefix;

    for (size_t j = 0; j < N; ++j) {
      double currNum = data[j];
      output += (j < N - 1) ? std::to_string(currNum).substr(0, numOfDigits + 1) + ", " : std::to_string(currNum).substr(0, numOfDigits + 1);
    }

    MainControl.print(row, 0, output.c_str(), 0);
  }
}

#pragma endregion