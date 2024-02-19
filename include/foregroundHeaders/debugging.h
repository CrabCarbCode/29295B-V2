#include "custPrinting.h"
#include "data-storage.h"
#include "main.h"

float adjustFactor = 0.05;  // the increment by which PID variables change during manual tuning
bool isTuningTurns = true;

// variables which control the shape/range of the acceleratory curve
float ACurveExtremity = 0.1996;  // sigma
float peakPos = 1;               // mu
float AMinAmount = 0.24;         // kappa

// variables which control the shape of the stick curve
float linearHarshness = 0.6;  // g on graph
float SCurveExtremity = 5.3;  // h on graph

void tunePID(bool isPrinting);

void tuneDrive(bool isPrinting);
#pragma endregion