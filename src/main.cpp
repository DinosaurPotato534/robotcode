#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/misc.h"


pros::Controller controller(pros::E_CONTROLLER_MASTER);

// drive motors
pros::Motor lF(-9, pros::E_MOTOR_GEARSET_06); // left front motor
pros::Motor lM(10, pros::E_MOTOR_GEARSET_06); // left middle motor
pros::Motor lB(-8, pros::E_MOTOR_GEARSET_06); // left back motor
pros::Motor rF(19, pros::E_MOTOR_GEARSET_06); // right front motor
pros::Motor rM(-20, pros::E_MOTOR_GEARSET_06); // right middle motor
pros::Motor rB(18, pros::E_MOTOR_GEARSET_06); // right back motor

pros::Motor intake(6, pros::E_MOTOR_GEAR_200, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor kicker1(16, pros::E_MOTOR_GEAR_200, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor kicker2(17, pros::E_MOTOR_GEAR_200, true, pros::E_MOTOR_ENCODER_DEGREES);

#define HANG_SOLENOID_PORT  'D'
#define BACK_WING_SOLENOID_PORT 'A'
#define RIGHT_WING_SOLENOID_PORT 'C'
#define LEFT_WING_SOLENOID_PORT 'B'

pros::ADIDigitalOut left_wing_solenoid(LEFT_WING_SOLENOID_PORT);
pros::ADIDigitalOut right_wing_solenoid(RIGHT_WING_SOLENOID_PORT);
pros::ADIDigitalOut back_wing_solenoid(BACK_WING_SOLENOID_PORT);
pros::ADIDigitalOut hang_solenoid(HANG_SOLENOID_PORT);


// motor groups
pros::MotorGroup leftMotors({lF, lM, lB}); // left motor group
pros::MotorGroup rightMotors({rF, rM, rB}); // right motor group

// Inertial Sensor on port 10
pros::Imu imu(5);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              10, // 10 inch track width
                              lemlib::Omniwheel::NEW_4, // using new 3.25" omnis
                              343, // drivetrain rpm is 343
                              2 // chase power is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(10, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            3, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             10, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
// note that in this example we use internal motor encoders (IMEs), so we don't pass vertical tracking wheels
lemlib::OdomSensors sensors(nullptr,
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

void leftWingControl(bool state) {
    left_wing_solenoid.set_value(state);
}

// Function to control the right wing solenoid
void rightWingControl(bool state) {
    right_wing_solenoid.set_value(state);
}

void backWingControl(bool state) {
    back_wing_solenoid.set_value(state);
}

void hangControl(bool state) {
    hang_solenoid.set_value(state);
}

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
   pros::Task screenTask([&]() {
        lemlib::Pose pose(0, 0, 0);
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
}
/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

ASSET(path_txt);

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
    chassis.follow(path_txt, 15, 4000, false);
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {

	while (true) {
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with curvature drive
        chassis.arcade(leftY, rightX);

// Read joystick inputs for intake control
        if (controller.get_digital(DIGITAL_R1)) {
            intake.move_velocity(200); 
        } else if (controller.get_digital(DIGITAL_R2)) {
            intake.move_velocity(-200);
        } else {
            intake.move_velocity(0);
        }

        // Read joystick inputs for kicker control
        if (controller.get_digital(DIGITAL_L2)) {
            kicker1.move_velocity(200);
            kicker2.move_velocity(200);
        } else {
            kicker1.move_velocity(0);
            kicker2.move_velocity(0);
        }

        // Read joystick inputs for wing control
        leftWingControl(controller.get_digital(DIGITAL_RIGHT));
        rightWingControl(controller.get_digital(DIGITAL_Y));
        backWingControl(controller.get_digital(DIGITAL_L1));
        hangControl(controller.get_digital(DIGITAL_A));

        if (controller.get_digital(DIGITAL_B)) {
            autonomous();
        }

        // delay to save resources
        pros::delay(10);
    }

}
