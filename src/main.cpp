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


int COLOR = 0; // 0 - 50 (red)
int OP_COLOR = 180; //110 - 220 (blue)
bool is_auto = false;
bool is_long = false;
bool is_top = false;
bool is_manual = false;

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

pros::Optical opticalSensor(20);
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

void auto_intake() {
	while (true) {
		int hue_value = opticalSensor.get_hue();
		pros::screen::erase();

		if (is_auto) {
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
		} else {
			intake_stop();
		}

		pros::delay(10);
	}
}

// Autonomous
void autonomous() {
	// Parked
	chassis.setPose(0, 0, 0);

	// Move to loader
	chassis.moveToPoint(0, 32, 10000);

	chassis.turnToHeading(-90, 1000);

	chassis.moveToPoint(-13, 29, 1000);

	pros::delay(2000);

	// Move to long score
	chassis.moveToPoint(0, 32, 4000, {.forwards = false});

	chassis.turnToHeading(90, 1000, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE});
	
	chassis.moveToPoint(20, 33, 1000);

	score_long();
	pros::delay(2000);
	intake_stop();

	chassis.turnToHeading(135, 1000);

	// Move to middle blocks
	chassis.moveToPoint(0, 29, 4000, {.forwards = false}, true);
	is_auto = true;

	chassis.moveToPoint(20, 20, 4000, {.minSpeed=70});

	// Move to middle score
	chassis.moveToPoint(40, 0, 100000000);
	pros::delay(1000);
	score_top();
	is_auto = false;
	//pros::delay(2000);
	//intake_stop();

}

void opcontrol() {
	pros::Task auto_task(auto_intake, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Auto Intake");
	auto_task.resume();

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
		}

		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
			is_long = !is_long;
		}

		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
			is_top = !is_top;
		}

		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
			is_manual = true;
			is_auto = false;
			is_long = false;
			is_top = false;
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

		//if (is_long) {
			//is_auto = false;
			//is_top = false;
			//score_long();
		//}
		if (is_top) {
			is_auto = false;
			is_long = false;
			score_top();
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
