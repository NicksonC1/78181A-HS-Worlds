// <--------------------------------------------------------------- Includes --------------------------------------------------------------->
#include <bits/stdc++.h>
#include <vector>
#include <functional>
#include <string>
// #include <iomanip>
#include "main.h"
#include "lebron/api.hpp" // IWYU pragma: keep
// #include "liblvgl/lvgl.h"
// #include "liblvgl/llemu.hpp"
#include "brainScreenLVGL.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"

// <--------------------------------------------------------------- Setup ------------------------------------------------------------------>
// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// DriveTrain
pros::MotorGroup leftMotors({-1, -2, -3}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({4, 5, 6}, pros::MotorGearset::blue); 

namespace Motor{
    pros::Motor intakeB(7, pros::MotorGearset::blue);
    pros::Motor intakeT(8, pros::MotorGearset::blue);
    pros::Motor lb(9, pros::MotorGearset::blue);
} // namespace Motor

namespace Sensor{
    pros::Rotation lbR(10);
    pros::Distance lbD(11); // Del
    pros::Optical colorSort(12);
    pros::adi::DigitalIn autonSwitch('A');
} // namspace Sensor

namespace Piston{
    pros::adi::DigitalOut lightsaberL('B');
    pros::adi::DigitalOut lightsaberR('C');
    pros::adi::DigitalOut intake('D');
    pros::adi::DigitalOut mogo('E');
    pros::adi::DigitalOut sorter('F');
    pros::adi::DigitalOut pto('G');
} // namespace Piston

// <------------------------------------------------------------- Odom Sensors ------------------------------------------------------------->
pros::Imu imu(2);
pros::Rotation horizontalEnc(-16);
pros::Rotation verticalEnc(-15);
lebron::TrackingWheel vertical_tracking_wheel(&verticalEnc, lebron::Omniwheel::NEW_2 , 0.9); // Single
lebron::TrackingWheel horizontal_tracking_wheel(&horizontalEnc, 2.0 , -2.2); // Double Stacked

// <---------------------------------------------------------------- Config ---------------------------------------------------------------->
lebron::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              11.5, // 11.5 inch track width
                              lebron::Omniwheel::NEW_325, // using new 3.25" omnis
                              450, // drivetrain rpm is 450
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

lebron::ControllerSettings linearController (8, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             5, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in inches
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in inches
                                             500, // large error range timeout, in milliseconds
                                             5 // maximum acceleration (slew)
);

lebron::ControllerSettings angularController (3, // proportional gain (kP)
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
lebron::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal_tracking_wheel, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lebron::ExpoDriveCurve throttleCurve(1, // joystick deadband out of 127
                                     0, // minimum output where drivetrain will move out of 127
                                     1 // expo curve gain
);

// input curve for steer input during driver control
lebron::ExpoDriveCurve steerCurve(1, // joystick deadband out of 127
                                  0, // minimum output where drivetrain will move out of 127
                                  1 // expo curve gain
);

// create the chassis
lebron::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

std::vector<std::pair<float, float>> points;

// <------------------------------------------------------------ Miscellaneous ------------------------------------------------------------>
namespace Misc{
    constexpr int DELAY = 10;
    void togglePiston(pros::adi::DigitalOut &piston, bool &state) {
        state = !state;
        piston.set_value(state);
    }
    void cdrift(float lV, float rV, int timeout, bool cst = true){
        (cst == true) ? (leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST), rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST)) : (leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE), rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE));
        leftMotors.move(lV);
        rightMotors.move(rV);
        pros::delay(timeout);
        leftMotors.brake();
        rightMotors.brake();
    }
    void curve(std::vector<std::pair<float, float>>& waypoints, int angular = 450, int lateral = 2300){
        while(!waypoints.empty()){
            std::pair<int, int> target = waypoints.front();
            chassis.turnToPoint(target.first,target.second,angular,{.minSpeed = 10,.earlyExitRange = 2});
            chassis.moveToPoint(target.first,target.second,lateral,{.minSpeed = 10,.earlyExitRange = 2});
            chassis.waitUntilDone();
            waypoints.erase(waypoints.begin());
        }
    }
    void linear(double dist, int timeout, lebron::MoveToPointParams p = {}, bool async = true){
        lebron::Pose pose = chassis.getPose(true);
        dist < 0 ? p.forwards = false : p.forwards = true;
        chassis.moveToPoint(
        pose.x + std::sin(pose.theta) * dist,
        pose.y + std::cos(pose.theta) * dist,
        timeout,
        p,
        async);
        // https://github.com/DHRA-2131/2024-25-2131H/blob/010ce1434e415ad07cc6ba06149c97fcf4c29a76/Rewrite%203/include/2131H/Systems/Chassis.hpp#L13
    }
} // namespace Misc

// <-------------------------------------------------------------- Lady Brown ------------------------------------------------------------>
namespace Lift{
    int target = 0, pressCounter = 0, curr = 0;
    constexpr int RESET = 0;
    constexpr int LOAD = 300;
    constexpr int FREE = 600;
    constexpr int SCORE = 2000;
    constexpr int STARTING = 150;
    std::vector<int> STATES = {RESET, LOAD, SCORE};
    // constexpr float STATES[] = {RESET, STARTING, LOAD, FREE, SCORE};
    constexpr int NUM_STATES = sizeof(STATES) / sizeof(STATES[0]);
    void setState(float state) { target = state; }
    void move() { 
        double ukp = 0.5, dkp = 0.3, error = target - Sensor::lbR.get_position(), vU = ukp * error, vD = dkp * error; 
        if(target == SCORE){
            if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
                Motor::lb.move(127);
                pressCounter = 0;
            }
            else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
                Motor::lb.move(-127);
                pressCounter++;
                if(Sensor::lbR.get_position() < 20){ setState(Lift::RESET); } // A
            }
            else{
                Motor::lb.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
                Motor::lb.brake();
                pressCounter = 0;
            }
            if(pressCounter > 50){ setState(Lift::RESET); }
        }
        if(std::fabs(error) < 5){ Motor::lb.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); Motor::lb.brake(); }
        else if(error> 0){ Motor::lb.move(vU); }
        else Motor::lb.move(vD);
        if(Sensor::lbR.get_position() < 20){ setState(Lift::RESET); } // A
    }
} //namespace Arm

// <------------------------------------------------------------- Color Sort ------------------------------------------------------------->
namespace Color{
    enum class colorVals{ BLUE, RED };
    constexpr bool sRed = true; // True: Sort Red  Color::sRed;
    constexpr bool sBlue = false; // False: Sort Blue Color::sBlue;
    bool state = false, extend_once = false;
    constexpr double rLow = 10.0, rHigh = 50.0, bLow = 164.0, bHigh = 213.5;
    bool ifSenseRed(int hue) { return (hue > rLow && Sensor::colorSort.get_hue() < rHigh) ?  true : false; }
    bool ifSenseBlue(int hue) { return (hue > bLow && Sensor::colorSort.get_hue() < bHigh) ? true : false; }
    void colorSortV2(colorVals input){
        while(1){
            int colorV = Sensor::colorSort.get_hue();
            if(input == colorVals::BLUE && ifSenseBlue(colorV) && !extend_once){
                Piston::sorter.set_value(true);
                pros::delay(350);
                extend_once = true;
            }
            else if(input == colorVals::RED && ifSenseRed(colorV) && !extend_once){
                Piston::sorter.set_value(true);
                pros::delay(350);
                extend_once = true;
            }
            else { Piston::sorter.set_value(false); }
            extend_once = false;
            pros::delay(10);
        }
    }
} // namespace Color

// <------------------------------------------------------------- Tier Three ------------------------------------------------------------->
namespace Hang{
    const int UNWRAP_TIME = 700;
    void reset(){ Misc::cdrift(-127,-127,UNWRAP_TIME); }
    void up(int iterE, int cd){ Misc::cdrift(127,127,iterE); pros::delay(cd); }
    void iter(int iterE, int cd, int times){ for(int i=0;i<times;i++){ up(iterE,cd); reset(); }}
    void move(int iterE, int cd){
        Piston::pto.set_value(true);
        Misc::cdrift(15,15,200); // Gear init
        iter(iterE, cd, 3);
    }
} // namespace Tier

// <-------------------------------------------------------------- Auto Routes ----------------------------------------------------------->
namespace Auton{
    int state = 0;
    void test() { Color::state = Color::sRed; }
    void coords(){ 
        points.emplace_back(-24,24);
        points.emplace_back(-7,41);
        points.emplace_back(24,48);
        Misc::curve(points); // vec, angular timeout, lateral timeout
    }
    void linear(){
        Misc::linear(24,2000,{.forwards = true,.maxSpeed = 127,.minSpeed = 10,.earlyExitRange = 2});
    }
} // namespace Auton

using AutonFunc = void(*)();
std::vector<std::pair<std::string, AutonFunc>> autonRoutines = {
    {"Default Auton", Auton::test},
    {"Coords Testing", Auton::coords}
};

void autonSelectSwitch() {
    while (1) {
        pros::delay(Misc::DELAY);
        if (Sensor::autonSwitch.get_new_press()) { Auton::state++; if (Auton::state >= autonRoutines.size()) Auton::state = 0; }
        pros::lcd::set_text(4, autonRoutines[Auton::state].first);
    }
}

Color::colorVals getColor(bool colorValV3) { return colorValV3 ? Color::colorVals::RED : Color::colorVals::BLUE; } // Sort ? Red : Blue

// <------------------------------------------------------------ Initialize --------------------------------------------------------------->
void initialize() {
    pros::lcd::initialize();
    pros::Task t_Select(autonSelectSwitch);
    pros::Task liftC([]{ while (1) { Lift::move(); pros::delay(Misc::DELAY); } });
    chassis.setPose(0, 0, 0);
    chassis.calibrate(); 
    Motor::intakeB.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    Motor::intakeT.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    Sensor::colorSort.set_led_pwm(100);
    pros::Task screenTask([&]() {
        while (1) {
            pros::lcd::print(0, "X: %f", chassis.getPose().x);
            pros::lcd::print(1, "Y: %f", chassis.getPose().y);
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
            pros::delay(50);
        }
    });
}
void disabled() {}
void competition_initialize() {}
ASSET(example_txt); // PP
// <------------------------------------------------------------- Autonomous ------------------------------------------------------------->
void autonomous() {
    pros::Task sorterC([&]() { colorSortV2(getColor(Color::state)); });
    Auton::coords();
    pros::delay(10000000);
    (Auton::state < autonRoutines.size()) ? autonRoutines[Auton::state].second() : Auton::test();
}

// <--------------------------------------------------------------- Driver --------------------------------------------------------------->
void opcontrol() {
    pros::Task sorterC([&]() { colorSortV2(getColor(Color::state)); });
    bool b_mogo = false, b_pto = false;
	leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
	rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    Sensor::lbR.reset_position();
    while(1) {
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        chassis.arcade(leftY, rightX);
		if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) { Misc::togglePiston(Piston::mogo, b_mogo); }
        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) { Misc::togglePiston(Piston::pto, b_pto); } // Testing | Del 
		(controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) ? Piston::lightsaberL.set_value(true) : Piston::lightsaberL.set_value(false);
        (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) ? Piston::lightsaberR.set_value(true) : Piston::lightsaberR.set_value(false);
        (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) ? Piston::intake.set_value(true) : Piston::intake.set_value(false);
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) { Motor::intakeB.move(127); Motor::intakeT.move(-127); }
        else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) { Motor::intakeB.move(-127); Motor::intakeT.move(-127); }
        else{ Motor::intakeB.brake(); Motor::intakeT.brake(); }
        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) { Lift::curr++; }
        else if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) { Lift::curr--; }
        if(Lift::curr > Lift::STATES.size()) { Lift::curr = 0;}
        Lift::setState(Lift::STATES[Lift::curr]);
        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) Hang::move(1500,100);
        pros::delay(Misc::DELAY);
    }
}