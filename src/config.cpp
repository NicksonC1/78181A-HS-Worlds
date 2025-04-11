#include "main.h"
#include "genesis/api.hpp"
#include "pros/rtos.hpp"
#include "pros/distance.hpp"
#include "pros/optical.hpp"
 // <--------------------------------------------------------------- Setup ------------------------------------------------------------------>
// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// DriveTrain
pros::MotorGroup leftMotors({-11, 12, -13}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({18, -19, 20}, pros::MotorGearset::blue); 

namespace Motor{
    // pros::Motor intakeB(16, pros::MotorGearset::blue);
    pros::Motor intakeT(-2, pros::MotorGearset::blue);
    pros::Motor lb(-6, pros::MotorGearset::green);
} // namespace Motor

namespace Sensor{
    // pros::Rotation lbR(7);
    pros::Distance hang(5);
    pros::Optical o_colorSort(9);
    pros::Optical d_colorSort(100);
    pros::adi::DigitalIn autonSwitch({8,'Z'});
} // namspace Sensor

namespace Piston{
    pros::adi::DigitalOut lightsaberL({8,'G'}); // checked
    // pros::adi::DigitalOut lightsaberR({8,'Z'});
    pros::adi::DigitalOut saberclamp('F'); // checked
    // pros::adi::DigitalOut intake({8,'Z'});
    pros::adi::DigitalOut mogo({8,'H'}); // checked
    pros::adi::DigitalOut sorter('H'); // checked
    pros::adi::DigitalOut pto('G'); // checked
} // namespace Piston

// <------------------------------------------------------------- Odom Sensors ------------------------------------------------------------->
class CustomIMU : public pros::IMU {
  public:
    CustomIMU(int port, double scalar)
      : pros::IMU(port),
        m_port(port),
        m_scalar(scalar) {}
    virtual double get_rotation() const {
      return pros::c::imu_get_rotation(m_port) * m_scalar;
    }
  private:
    const int m_port;
    const double m_scalar;
};

CustomIMU s_imu(21, 1.01);
// pros::Imu imu(21);
pros::Rotation horizontalEnc(-17);
pros::Rotation verticalEnc(15);
genesis::TrackingWheel vertical_tracking_wheel(&verticalEnc, 2.0 , -0.9); // Single
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
                                             0, // derivative gain (kD)
                                             0, // anti windup
                                             0, // small error range, in inches
                                             0, // small error range timeout, in milliseconds
                                             0, // large error range, in inches
                                             0, // large error range timeout, in milliseconds
                                             5 // maximum acceleration (slew)
);

genesis::ControllerSettings angularController(2.85, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              14, // derivative gain (kD) 
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// genesis::ControllerSettings angularController(0, // proportional gain (kP)
//                                               0, // integral gain (kI)
//                                               0, // derivative gain (kD) 
//                                               0, // anti windup
//                                               0, // small error range, in inches
//                                               0, // small error range timeout, in milliseconds
//                                               0, // large error range, in inches
//                                               0, // large error range timeout, in milliseconds
//                                               0 // maximum acceleration (slew)
// );

// genesis::ControllerSettings linearController(8, // proportional gain (kP)
//                                              0, // integral gain (kI)
//                                              5, // derivative gain (kD)
//                                              3, // anti windup
//                                              1, // small error range, in inches
//                                              100, // small error range timeout, in milliseconds
//                                              3, // large error range, in inches
//                                              500, // large error range timeout, in milliseconds
//                                              5 // maximum acceleration (slew)
// );

// genesis::ControllerSettings angularController(3, // proportional gain (kP)
//                                               0, // integral gain (kI)
//                                               15, // derivative gain (kD)
//                                               3, // anti windup
//                                               1, // small error range, in inches
//                                               100, // small error range timeout, in milliseconds
//                                               3, // large error range, in inches
//                                               500, // large error range timeout, in milliseconds
//                                               0 // maximum acceleration (slew)
// );

// sensors for odometry
genesis::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal_tracking_wheel, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &s_imu // inertial sensor
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