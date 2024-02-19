#include "custPrinting.h"
#include "main.h"
#include "robot-config.h"


void lcdControl() {
  if (globalTimer % 11 == 0) {  // refresh the screen every 11 ticks because 11 is a good number :)
    MainControl.clear();
  }

  if (MainControl.get_digital_new_press(DIGITAL_LEFT) && currentPage > 0) { currentPage--; }
  if (MainControl.get_digital_new_press(DIGITAL_RIGHT)) { currentPage++; }
}

int pageRangeFinder(int index) {                 // calculates which page(s) a
  int startingPage = isPrintingList[0] ? 1 : 0;  // given function should print to
  // start on page 1
  for (int j = 0; j < index; ++j) { startingPage += isPrintingList[j] ? pagesPerPrint[j] : 0; }

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