#include "pros/screen.hpp"
#include <cstddef>
#include <iostream>
using namespace std;

#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "liblvgl/display/lv_display.h"
#include "liblvgl/llemu.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"


int COLOR = 190;
int OP_COLOR = 0;

// Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Motors
pros::MotorGroup leftMotors({-20, 10, -19},
							pros::MotorGearset::blue);
pros::MotorGroup rightMotors({-1, 12, 11},
							pros::MotorGearset::blue);
pros::Motor firstIntake(16, pros::MotorGear::blue);
pros::Motor secondIntake(15, pros::MotorGear::blue);

// Inertial
pros::Imu imu(17);

// Tracking wheels
pros::Rotation horizontalEnc(std::nullptr_t);
pros::Rotation verticalEnc(std::nullptr_t);

pros::Optical opticalSensor(14);
int hue_value = opticalSensor.get_hue();

//lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_325, 10);
//lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_325, 10);

// Drivetrain
lemlib::Drivetrain drivetrain(&leftMotors,
							  &rightMotors,
							  10,
							  lemlib::Omniwheel::NEW_325,
							  360,
							  2
);

// Lateral motion
lemlib::ControllerSettings linearController(13,
											0,
											5,
											3,
											1,
											100,
											3,
											500,
											20
);

// Angular motion
lemlib::ControllerSettings angularController(3,
											0,
											15,
											3,
											1,
											100,
											3,
											500,
											0
);

// Odometry Sensors
lemlib::OdomSensors sensors(nullptr,
							nullptr,
							nullptr,
							nullptr,
							&imu
);

// Throttle
lemlib::ExpoDriveCurve throttleCurve(3,
									10,
									1.019
);

// Curve
lemlib::ExpoDriveCurve steerCurve(3,
								  10,
								  1.019
);

// Chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);


// Initialization
void initialize() {
	pros::lcd::initialize();
	chassis.calibrate();
}

// Disabled
void disabled() {}

// Competition initialization
void competition_initialize() {}

void intake_drop() {
	firstIntake.move(-127);
	secondIntake.move(-127);
}

void intake() {
	firstIntake.move(-127);
	secondIntake.move(127);
}

void first_intake_out() {
	firstIntake.move(127);
}

void second_intake_out() {
	secondIntake.move(-127);
}

void intake_stop() {
	secondIntake.move(0);
	firstIntake.move(0);
}

void auto_intake() {
	hue_value = opticalSensor.get_hue();
	pros::screen::erase();
	pros::screen::print(pros::E_TEXT_MEDIUM, 4, "Hue value: %lf \n", hue_value);
	//pros::screen::print(TEXT_MEDIUM, 3, "%d", hue_value);
	if (hue_value > COLOR - 30 && hue_value < COLOR + 30) {
		intake();
	} else if (hue_value > OP_COLOR - 30 && hue_value < COLOR + 30) {
		intake_drop();
	} else {
		intake_stop();
	}
}

// Autonomous
void autonomous() {
	// Parked
	chassis.setPose(0, 0, 0);

	// Move to loader
	chassis.moveToPoint(0, 29, 100000000);

	chassis.turnToHeading(-90, 10000000);

	chassis.moveToPoint(-15, 29, 100000000);

	// Move to long score
	chassis.moveToPoint(0, 29, 4000, {.forwards = false}, true);

	chassis.moveToPoint(20, 29, 100000000);

	// Move to middle blocks
	chassis.moveToPoint(0, 29, 4000, {.forwards = false}, true);

	// Move to middle score
	chassis.moveToPoint(45, 0, 100000000);
}

void opcontrol() {

	while (true) {
		int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
		
		chassis.arcade(leftY, rightX);

		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
			intake();
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
			intake_drop();
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
			intake_stop();
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
			first_intake_out();
		} else {
			intake_stop();
		};

		auto_intake();

		pros::delay(10);
	}
}

/*
motor_group.move(voltage: number)
motor_group.move_velocity(velocity: number)
motor_group.brake()

controller.get_digital_new_press - true if button is down once
controller.get_digital_new_release - true if button is released
move(specific voltage level: max is 127/-127)
move_velocity(specific rpm) (consistent speed, not based on battery, for intake, etc.)
move_absolute(degree, velocity) (move specific # degrees from 0 @ certain speed)
move_relative(Degree, velocity) (move specific # degrees from where it is now at certain speed)
brake
set_break_mode(hold, coast, brake) (hold, free, quick)

controller.get_analog() - get joystick position
get_digital - 
returns true if button is held down
print(line, col, text)

distance.get() (distance sensor, number of mm)
optical.get_hue(), optical.get_proximity()
rotation.get_position(), rotation.reset_position()

*/
