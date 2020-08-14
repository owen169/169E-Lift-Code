#include "main.h"
using namespace okapi;

void initialize() {
	pros::lcd::initialize();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {

	// Initialize the Chassis with Odometry
	std::shared_ptr<OdomChassisController> chassis =
	  ChassisControllerBuilder()
	    .withMotors({14, 16}, {15, 17}) // Left: 14,16 | Right: 15, 17
	    .withDimensions(AbstractMotor::gearset::green, {{4.32_in, 12.25_in}, imev5GreenTPR}) // Drop Center Wheels: 4.32in
	    .withSensors(ADIEncoder{'A', 'B', true}, ADIEncoder{'C', 'D'}) // Left Tracking Pod: 'A', 'B' | Right Tracking Pod: 'C', 'D'
	    .withOdometry({{2.75_in, 6.25_in}, quadEncoderTPR}) // Pods: 2.75" Omni Wheel
	    .buildOdometry();

	// Initialize Motion Profiling
	std::shared_ptr<AsyncMotionProfileController> profileController =
	  AsyncMotionProfileControllerBuilder()
	    .withLimits({
	    	1.0, // Maximum linear velocity of the Chassis in m/s
	    	2.0, // Maximum linear acceleration of the Chassis in m/s/s
	    	10.0 // Maximum linear jerk of the Chassis in m/s/s/s
	    })
	    .withOutput(chassis)
	    .buildMotionProfileController();

	// Initialize the Lift
	std::shared_ptr<AsyncPositionController<double, double>> liftController = 
	  AsyncPosControllerBuilder()
    	.withMotor(7) // Lift Motor: Port 7
    	.withGains({0.001, 0.001, 0.0000}) // kP, kI, kD
		.withSensor(std::make_shared<okapi::Potentiometer>(7)) // Potentiometer: ADI Port 'G'
    	.build();

	// Set point to 0, 0
	chassis->setState({0_in, 0_in, 0_deg});

	// START GOAL 1

	topRoller.move(127); // Spin Top Roller
	liftController->setTarget(2100); // Raise Lift

	leftIntake.move(127); // Spin both side rollers
	rightIntake.move(127);

	profileController->generatePath(
    	{{0_ft, 0_ft, 0_deg}, {3_ft, 1_ft, 0_deg}}, "Goal1Turn1");
	profileController->setTarget("Goal1Turn1"); // Drive up to the first goal, picking up a red ball
	profileController->waitUntilSettled(); // Wait until finished

	chassis->turnToAngle(10_deg); // Turn to align left roller with the bottom ball

	profileController->generatePath(
    	{{0_ft, 0_ft, 0_deg}, {1.4_ft, 0_ft, 0_deg}}, "Goal1Straight1");
	profileController->setTarget("Goal1Straight1"); // Drive into the goal, pushing out the bottom ball
	profileController->waitUntilSettled(); // Wait until finished

	leftIntake.move(0); // Stop both side rollers
	rightIntake.move(0); 

	chassis->turnToAngle(-20_deg); // Turn so the top roller is aligned with the first goal

	topRoller.move(-127); // Drop the ball into the first goal -- GOAL 1 DONE

	pros::delay(1000); // Spin the top roller for 1 second

	topRoller.move(0); // Stop the top roller

	leftIntake.move(50); // Spin both side rollers in slowly
	rightIntake.move(50);

	profileController->generatePath(
    	{{0_ft, 0_ft, 0_deg}, {-2_ft, 0_ft, 0_deg}}, "Goal1Backup1");
	profileController->setTarget("Goal1Backup1", true); // Drive back from the goal
	profileController->waitUntilSettled(); // Wait until finished

	leftIntake.move(127); // Spin all rollers in
	rightIntake.move(127);
	topRoller.move(127);

	liftController->setTarget(700); // Lower the lift, transferring the ball in the bottom tray to the top tray

	chassis->driveToPoint({2_ft, 0_ft}); // Use odometry to realign in front of the starting point

	chassis->turnToAngle(90_deg); // Use odometry to turn 90 degrees from the starting point

	liftController->setTarget(1000); // Raise the lift slightly

	profileController->generatePath(
    	{{0_ft, 0_ft, 0_deg}, {2.5_ft, -1.5_ft, -45_deg}}, "Goal2Turn1");
	profileController->setTarget("Goal2Turn1"); // Drive to the second goal
	profileController->waitUntilSettled(); // Wait until finished

	liftController->setTarget(2000); // Raise the lift

	leftIntake.move(50); // Spin both side rollers in slowly
	rightIntake.move(50);

	chassis->turnToAngle(127_deg); // Use odometry to face the second goal

	profileController->generatePath(
    	{{0_ft, 0_ft, 0_deg}, {1.2_ft, 0_ft, 0_deg}}, "Goal2Straight1");
	profileController->setTarget("Goal2Straight1"); // Drive into the second goal
	profileController->waitUntilSettled(); // Wait until finished

	topRoller.move(-127); // Drop the ball into the second goal -- GOAL 2 DONE

	pros::delay(1000); // Spin the top roller for 1 second

	topRoller.move(0); // Stop the top roller

	leftIntake.move(-50); // Spin both side rollers out slowly
	rightIntake.move(-50);

	profileController->generatePath(
    	{{0_ft, 0_ft, 0_deg}, {-2.5_ft, 0_ft, 0_deg}}, "Goal2Backup1");
	profileController->setTarget("Goal2Backup1", true); // Drive back from the goal
	profileController->waitUntilSettled(); // Wait until finished

	liftController->setTarget(900); // Lower the lift

	chassis->turnToAngle(85_deg); // Use odometry to face the wall

	leftIntake.move(127); // Spin all rollers in
	rightIntake.move(127);
	topRoller.move(127);

	profileController->generatePath(
    	{{0_ft, 0_ft, 0_deg}, {1.75_ft, 0_ft, 0_deg}}, "Goal3Straight1");
	profileController->setTarget("Goal3Straight1"); // Drive to the wall, picking up the third ball
	profileController->waitUntilSettled(); // Wait until finished

	pros::delay(1000); // Leave 1 second to pick up the ball

	profileController->generatePath(
    	{{0_ft, 0_ft, 0_deg}, {-2_ft, 0_ft, 0_deg}}, "Goal3Backup1");
	profileController->setTarget("Goal3Backup1", true); // Backup from the wall
	profileController->waitUntilSettled(); // Wait until finished

	chassis->turnToAngle(0_deg); // Use odometry to face straight forward

	profileController->generatePath(
    	{{0_ft, 0_ft, 0_deg}, {2_ft, 0_ft, 0_deg}, {4_ft, 2_ft, 90_deg}}, "Goal3Turn1");
	profileController->setTarget("Goal3Turn1"); // Drive in an arc to the third goal
	profileController->waitUntilSettled(); // Wait until finished

	chassis->driveToPoint({101.5_in, -56_in}); // Use odometry to drive to the third goal

	chassis->turnToAngle(-45_deg); // Use odometry to face the third goal

	liftController->setTarget(2200); // Raise the lift

	profileController->generatePath(
    	{{0_ft, 0_ft, 0_deg}, {2_ft, 0_ft, 0_deg}}, "Goal3Straight2");
	profileController->setTarget("Goal3Straight2"); // Drive into the third goal
	profileController->waitUntilSettled(); // Wait until finished

	topRoller.move(-127); // Drop the ball into the third goal

	pros::delay(1000); // Spin the top roller for 1 second

	profileController->generatePath(
    	{{0_ft, 0_ft, 0_deg}, {-2_ft, 0_ft, 0_deg}}, "Goal3Backup2");
	profileController->setTarget("Goal3Backup2", true); // Backup from the third goal
	profileController->waitUntilSettled(); // Wait until finished

}

void opcontrol() {

	pros::ADIAnalogIn arm_poten(7); // Arm Potentiomoeter: ADI Port 'G'

	// Initialize the Lift
	std::shared_ptr<AsyncPositionController<double, double>> liftController = 
	  AsyncPosControllerBuilder()
    	.withMotor(7) // Lift Motor: Port 7
    	.withGains({0.001, 0.001, 0.0000}) // kP, kI, kD
		.withSensor(std::make_shared<okapi::Potentiometer>(7)) // Potentiometer: ADI Port 'G'
    	.build();

	while (true) {
	    
		pros::lcd::print(2, "arm %d", arm_poten.get_value()); // Print the arm value to the LCD

		// Drive Code
		leftBackDrive.move(controller.get_analog(ANALOG_LEFT_Y));
		leftFrontDrive.move(controller.get_analog(ANALOG_LEFT_Y));
		rightBackDrive.move(controller.get_analog(ANALOG_RIGHT_Y));
    	rightFrontDrive.move(controller.get_analog(ANALOG_RIGHT_Y));

		// Side Rollers
		if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1) == 1){
			leftIntake.move(127);
			rightIntake.move(127);
		} else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2) == 1) {
			leftIntake.move(-127);
			rightIntake.move(-127);
		} else {
			leftIntake.move(0);
			rightIntake.move(0);
		}

		// Top Rollers
		if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1) == 1){
			topRoller.move(127);
		} else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2) == 1) {
			topRoller.move(-127);
		} else {
			topRoller.move(0);
		}

		// Lift
		if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)==1){
			liftController->setTarget(1025);
		} else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)==1){
			liftController->setTarget(1500);
		} else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)==1){
			liftController->setTarget(2000);
		} else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)==1){
			liftController->setTarget(2400);
		} else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)==1){
			liftController->setTarget(800);
		}

		pros::delay(20);
	}
}
