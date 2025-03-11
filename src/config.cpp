#include "genesis/api.hpp"
#include "pros/rtos.hpp"
#include "pros/distance.hpp"
#include "pros/optical.hpp"
 // <--------------------------------------------------------------- Setup ------------------------------------------------------------------>
// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// DriveTrain
pros::MotorGroup leftMotors({-100, -100, -100}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({100, 100, 100}, pros::MotorGearset::blue); 

namespace Motor{
    pros::Motor intakeB(9, pros::MotorGearset::blue);
    pros::Motor intakeT(-4, pros::MotorGearset::blue);
    pros::Motor lb(100, pros::MotorGearset::green);
} // namespace Motor

namespace Sensor{
    pros::Rotation lbR(100);
    pros::Distance lbD(100); // Del
    pros::Optical colorSort(100);
    pros::adi::DigitalIn autonSwitch('Z');
} // namspace Sensor

namespace Piston{
    pros::adi::DigitalOut lightsaberL('Z');
    pros::adi::DigitalOut lightsaberR('Z');
    pros::adi::DigitalOut intake('Z');
    pros::adi::DigitalOut mogo('Z');
    pros::adi::DigitalOut sorter('Z');
    pros::adi::DigitalOut pto('Z');
} // namespace Piston

// <------------------------------------------------------------- Odom Sensors ------------------------------------------------------------->
pros::Imu imu(100);
pros::Rotation horizontalEnc(-100);
pros::Rotation verticalEnc(-100);
genesis::TrackingWheel vertical_tracking_wheel(&verticalEnc, genesis::Omniwheel::NEW_2 , -0.9); // Single
genesis::TrackingWheel horizontal_tracking_wheel(&horizontalEnc, 2.0 , -2.2); // Double Stacked

// <---------------------------------------------------------------- Config ---------------------------------------------------------------->
genesis::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              11.5, // 11.5 inch track width
                              genesis::Omniwheel::NEW_325, // using new 3.25" omnis
                              450, // drivetrain rpm is 450
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

genesis::ControllerSettings linearController(8, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             5, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in inches
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in inches
                                             500, // large error range timeout, in milliseconds
                                             5 // maximum acceleration (slew)
);

genesis::ControllerSettings angularController(3, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              15, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// sensors for odometry
genesis::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal_tracking_wheel, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
genesis::ExpoDriveCurve throttleCurve(1, // joystick deadband out of 127
                                     0, // minimum output where drivetrain will move out of 127
                                     1 // expo curve gain
);

// input curve for steer input during driver control
genesis::ExpoDriveCurve steerCurve(1, // joystick deadband out of 127
                                  0, // minimum output where drivetrain will move out of 127
                                  1 // expo curve gain
);

// create the chassis
genesis::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);