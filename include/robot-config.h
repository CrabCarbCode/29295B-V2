#pragma once
#include "main.h"


using namespace pros;


/*
 * note on name conventions, every variable is camelcase
 * while all components/functions have capitalized first words
 * motors end with M
 */
pros::Controller MainControl(E_CONTROLLER_MASTER);  // declared with "pros::" to avoid ambiguity with stock vex controller
pros::Controller SideControl(E_CONTROLLER_PARTNER);


Link PrimaryRadio(21, "S(1)Car", E_LINK_TRANSMITTER);
Link SecondaryRadio(5, "S(2)Car", E_LINK_TRANSMITTER);


Motor LDriveFrontM(12, E_MOTOR_GEARSET_06, true, E_MOTOR_ENCODER_DEGREES);
Motor LDriveMidM(14, E_MOTOR_GEARSET_06, true, E_MOTOR_ENCODER_DEGREES);
Motor LDriveBackM(13, E_MOTOR_GEARSET_18, true, E_MOTOR_ENCODER_DEGREES);  // 5.5w


Motor RDriveFrontM(20, E_MOTOR_GEARSET_06, false, E_MOTOR_ENCODER_DEGREES);
Motor RDriveMidM(19, E_MOTOR_GEARSET_06, false, E_MOTOR_ENCODER_DEGREES);
Motor RDriveBackM(15, E_MOTOR_GEARSET_18, false, E_MOTOR_ENCODER_DEGREES);  // 5.5w


Motor LiftM(18, E_MOTOR_GEARSET_36, false, E_MOTOR_ENCODER_DEGREES);


Motor IntakeM(10, E_MOTOR_GEARSET_06, false, E_MOTOR_ENCODER_DEGREES);
Motor KickerM(2, E_MOTOR_GEARSET_36, false, E_MOTOR_ENCODER_DEGREES);


ADIDigitalOut WingPR('G');
ADIDigitalOut WingPL('H');


Imu Inertial(16);  // initializing the Inertial sensor
Rotation ArmRot(8);
Rotation LOdem(20);
Rotation ROdem(19);
Rotation SOdem(18);



Motor_Group LDrive600({LDriveFrontM, LDriveMidM});
Motor_Group RDrive600({RDriveFrontM, RDriveMidM});


Motor_Group FullDrive({LDriveFrontM, LDriveMidM, LDriveBackM, RDriveBackM, RDriveMidM, RDriveBackM});