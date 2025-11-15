#include "pros/colors.h"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include "pros/screen.h"
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
#include "api.h"

// 0 = RED
// 180 = BLUE

int COLOR = 180; // 0 - 50 (red) 
int OP_COLOR = 0; //110 - 220 (blue)
bool is_auto = false;
bool is_long = false;
bool is_top = false;
bool is_bottom = false;
bool is_manual = false;
bool is_move = false;

// Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Motors
pros::MotorGroup leftMotors({-20, 10, -19},
							pros::MotorGearset::blue);
pros::MotorGroup rightMotors({-1, 12, 11},
							pros::MotorGearset::blue);
pros::Motor firstMotor(16, pros::MotorGear::green);
pros::Motor secondMotor(8, pros::MotorGear::green);
pros::Motor thirdMotor(5, pros::MotorGear::green);
pros::Motor fourthMotor(4, pros::MotorGear::green);

// Inertial
pros::Imu imu(17);

// Tracking wheels
pros::Rotation horizontalEnc(std::nullptr_t);
pros::Rotation verticalEnc(std::nullptr_t);

pros::Optical opticalSensor(18);
pros::Optical opticalSensor2(3);

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
	opticalSensor.set_led_pwm(50);
}

// Disabled
void disabled() {}

// Competition initialization
void competition_initialize() {}

void intake_drop() {
	firstMotor.move(-127);
	secondMotor.move(-127);
}

void intake() {
	firstMotor.move(-127);
	secondMotor.move(127);
	thirdMotor.move(-127);
}

void intake_stop() {
	secondMotor.move(0);
	firstMotor.move(0);
	thirdMotor.move(0);
	fourthMotor.move(0);
}

void score_long() {
	firstMotor.move(-127);
	secondMotor.move(-127);
	thirdMotor.move(127);
	fourthMotor.move(-127);
}

void score_top() {
	firstMotor.move(-127);
	secondMotor.move(-127);
	thirdMotor.move(127);
	fourthMotor.move(127);	
}

void score_bottom() {
	firstMotor.move(127);
	secondMotor.move(0);
	thirdMotor.move(127);
	fourthMotor.move(0);
}

void auto_intake() {
	while (true) {
		int hue_value = opticalSensor.get_hue();
		pros::screen::erase();

		if (is_auto) {
			intake();
		
			if (hue_value > OP_COLOR - 20 && hue_value < OP_COLOR + 20) {
				intake_drop();
				pros::screen::set_pen(pros::c::COLOR_BLUE);
				pros::screen::fill_rect(1,1,480,200);	
				pros::screen::print(pros::E_TEXT_MEDIUM, 3, "Hue: %i", hue_value);			
				pros::delay(800);
			} else if (hue_value > COLOR - 30 && hue_value < COLOR + 30) {
				pros::screen::set_pen(pros::c::COLOR_RED);
				pros::screen::fill_rect(1,1,480,200);
				pros::screen::print(pros::E_TEXT_MEDIUM, 3, "Hue: %i", hue_value);					
				intake();
			} else {
				//pros::screen::set_pen(pros::c::COLOR_BLACK);
				//pros::screen::fill_rect(1,1,480,200);	
				pros::screen::print(pros::E_TEXT_MEDIUM, 3, "Hue: %i", hue_value);	
				intake();							
			}
		} else if (!is_move) {
			intake_stop();
		}

		pros::delay(10);
	}
}

void auto_score() {
	while (true) {
		if (is_long) {
			score_long();
		} else if (is_top) {
			score_top();
		} else if (is_bottom) {
			score_bottom();
		} else if (!is_move) {
			intake_stop();
		}

		pros::delay(10);
	}
}

// Autonomous
void autonomous() {
	pros::Task autoIntake(auto_intake, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Auto Intake");
	autoIntake.resume();

	pros::Task autoScore(auto_score, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Auto Score");
	autoScore.resume();

	// Parked
	chassis.setPose(0, 0, 0);

	// Move to loader
	chassis.moveToPoint(0, 30, 10000);

	chassis.turnToHeading(-90, 1000);

	chassis.moveToPoint(-13, 30, 1000);

	//pros::delay(2000);

	// Move to long score
	chassis.moveToPoint(0, 30, 4000, {.forwards = false});

	chassis.turnToHeading(90, 1000, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE});
	
	chassis.moveToPoint(18, 31, 1000);

	score_long();
	pros::delay(2000);
	intake_stop();

	// Move to middle blocks
	chassis.moveToPoint(0, 30, 4000, {.forwards = false}, true);

	chassis.turnToHeading(135, 1000);
	is_auto = true;

	chassis.moveToPoint(40, 0, 4000, {.maxSpeed=70});

	// Move to middle score
	pros::delay(2000);
	is_auto = false;
	is_top = true;
	//pros::delay(2000);
	//intake_stop();

}

void opcontrol() {
	pros::Task autoIntake(auto_intake, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Auto Intake");
	autoIntake.resume();

	pros::Task autoScore(auto_score, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Auto Score");
	autoScore.resume();

	int first_motor_movement = 127;
	int second_motor_movement = 127;
	int third_motor_movement = 127;

	while (true) {
		int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
		
		chassis.arcade(leftY, rightX);

		opticalSensor.set_led_pwm(100);

		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
			is_auto = !is_auto;
			if (is_auto) {
				intake_stop();
				is_long = false;
				is_top = false;
				is_bottom = false;
			}
		}

		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
			is_long = !is_long;
			if (is_long) {
				intake_stop();
				is_auto = false;
				is_top = false;
				is_bottom = false;
			}
		}

		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
			is_top = !is_top;
			if (is_top) {
				intake_stop();
				is_long = false;
				is_auto = false;
				is_bottom = false;
			}
		}

		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
			is_bottom = !is_bottom;
			if (is_bottom) {
				intake_stop();
				is_top = false;
				is_auto = false;
				is_long = false;
			}
		}

		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
			is_manual = true;
			is_auto = false;
			is_long = false;
			is_top = false;
			is_bottom = false;
		}

		while (is_manual) {
			if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
				first_motor_movement = -first_motor_movement;
				second_motor_movement = -second_motor_movement;
				third_motor_movement = -third_motor_movement;
			}

			if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
				firstMotor.move(first_motor_movement);
			}

			if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
				secondMotor.move(second_motor_movement);
			}

			if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
				thirdMotor.move(third_motor_movement);
			}

			if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
				is_manual = false;
			}
		}

		if (is_auto || is_long || is_top || is_bottom || is_manual) {
			is_move = true;
		} else {
			is_move = false;
		}

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
