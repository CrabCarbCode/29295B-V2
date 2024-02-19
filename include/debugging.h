#include "custPrinting.h"
#include "data-storage.h"
#include "main.h"

float adjustFactor = 0.05;  // the increment by which PID variables change during manual tuning
bool isTuningTurns = true;

void tunePID(bool isPrinting);

void tuneDrive(bool isPrinting);
#pragma endregion