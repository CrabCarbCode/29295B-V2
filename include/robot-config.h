#include "main.h"

/*
 * note on name conventions, every variable is camelcase
 * while all components/functions have capitalized first words
 * motors end with M
 */
pros::Controller MainControl(E_CONTROLLER_MASTER);  // declared with "pros::" to avoid ambiguity with stock vex controller


Motor LDriveFrontM(5, E_MOTOR_GEARSET_06, false, E_MOTOR_ENCODER_DEGREES);
Motor LDriveMidM(6, E_MOTOR_GEARSET_06, false, E_MOTOR_ENCODER_DEGREES);
Motor LDriveBackM(9, E_MOTOR_GEARSET_18, true, E_MOTOR_ENCODER_DEGREES);  // 5.5w

Motor RDriveFrontM(3, E_MOTOR_GEARSET_06, true, E_MOTOR_ENCODER_DEGREES);
Motor RDriveMidM(7, E_MOTOR_GEARSET_06, true, E_MOTOR_ENCODER_DEGREES);
Motor RDriveBackM(8, E_MOTOR_GEARSET_18, false, E_MOTOR_ENCODER_DEGREES);  // 5.5w

Motor ArmM(10, E_MOTOR_GEARSET_36, false, E_MOTOR_ENCODER_DEGREES);

Motor IntakeM(15, E_MOTOR_GEARSET_18, false, E_MOTOR_ENCODER_DEGREES);

ADIDigitalOut WingPR('G');
ADIDigitalOut WingPL('H');

Imu Inertial(4);  // initializing the Inertial sensor
Rotation LOdem(20);
Rotation ROdem(19);
Rotation SOdem(18);

Motor_Group LDrive600({LDriveFrontM, LDriveMidM});
Motor_Group RDrive600({RDriveFrontM, RDriveMidM});

Motor_Group FullDrive({LDriveFrontM, LDriveMidM, LDriveBackM, RDriveBackM, RDriveMidM, RDriveBackM});