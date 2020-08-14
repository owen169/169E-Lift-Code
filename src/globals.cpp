#include "main.h"

//Motors

pros::Motor leftBackDrive(14, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor leftFrontDrive(16, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor rightFrontDrive(15, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor rightBackDrive(17, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_COUNTS);

pros::Motor lift(7, pros::E_MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_COUNTS);

pros::Motor leftIntake(5, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor rightIntake(6, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);

pros::Motor topRoller(4, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_COUNTS);

//Controller

pros::Controller controller(pros::E_CONTROLLER_MASTER);
