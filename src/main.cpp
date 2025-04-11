// <--------------------------------------------------------------- Includes --------------------------------------------------------------->
#include <bits/stdc++.h>
#include <vector>
#include <functional>
#include <string>
// #include <iomanip>
#include "main.h"
#include "genesis/api.hpp" // IWYU pragma: keep
#include "liblvgl/lvgl.h" //.
#include "liblvgl/llemu.hpp" //.
#include "brainScreenLVGL.h"
#include "config.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"

std::vector<std::pair<float, float>> points;
// Auton::state = 0;

namespace TaskHandler {
    bool colorSort = true;
    bool lb = true;
} // TaskHandler

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
    void chain(std::vector<std::pair<float, float>>& waypoints, int angular = 450, int lateral = 2300){
        while(!waypoints.empty()){
            std::pair<int, int> target = waypoints.front();
            chassis.turnToPoint(target.first,target.second,angular,{.minSpeed = 10,.earlyExitRange = 2});
            chassis.moveToPoint(target.first,target.second,lateral,{.minSpeed = 10,.earlyExitRange = 2});
            chassis.waitUntilDone();
            waypoints.erase(waypoints.begin());
        }
    }
    void linear(double dist, int timeout, genesis::MoveToPointParams p = {}, bool async = true){
        genesis::Pose pose = chassis.getPose(true);
        dist < 0 ? p.forwards = false : p.forwards = true;
        chassis.moveToPoint(
        pose.x + std::sin(pose.theta) * dist,
        pose.y + std::cos(pose.theta) * dist,
        timeout, p, async);
    }
    // https://github.com/DHRA-2131/2024-25-2131H/blob/010ce1434e415ad07cc6ba06149c97fcf4c29a76/Rewrite%203/include/2131H/Systems/Chassis.hpp#L13
    // void driver(bool mode, int I, int C, int deadband){
    //     double mL, mR, ret = 0;
    //     if(mode){ ret = (std::exp(-C/10)+std::exp((std::abs(I)-100)/10)(1-std::exp(-C/10))) I; }

    //     chassis.arcade(mL,mR);
    // }
} // namespace Misc

// <-------------------------------------------------------------- Lady Brown ------------------------------------------------------------>
namespace LiftSix {
    int lift_state = 0, pressCounter = 0;
    bool autonomousMode = false;

    constexpr int RESET = 0, LOAD = 175, SCORE = 950;
    std::vector<int> l_rot_target = {RESET, LOAD, SCORE};

    double lift_kp_up = 1.15, lift_kp_down = 0.35, lift_kd = 0;
    static double lastError = 0, lastTime = pros::millis();

    void setState(int newState) {
        lift_state = newState;
        autonomousMode = true;
        std::cout << "[Auto] SetState called: Lift state = " << lift_state << std::endl;
    }

    void autolift() {
        if (!autonomousMode) {
            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
                if (lift_state < 2) lift_state++;
                controller.rumble(".");
                std::cout << "[Driver] L2 Pressed: Lift state = " << lift_state << std::endl;
            }
            else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
                if (lift_state > 0) lift_state--;
                std::cout << "[Driver] R2 Pressed: Lift state = " << lift_state << std::endl;
            }
        }

        double motorPosition = Motor::lb.get_position();
        double error = l_rot_target[lift_state] - motorPosition;
        double deltaTime = std::max((pros::millis() - lastTime) / 1000.0, 0.01);
        double derivative = (error - lastError) / deltaTime;
        double velocity = (error > 0) ? (lift_kp_up * error) : (lift_kp_down * error);
        velocity += lift_kd * derivative;

        if (std::fabs(error) < 5) {
            Motor::lb.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            Motor::lb.brake();
        } else {
            Motor::lb.move(velocity);
        }

        if (motorPosition < 5) Motor::lb.tare_position();

        lastError = error;
        lastTime = pros::millis();

        // Add back later
        // controller.set_text(0, 0, "State: " + std::to_string(lift_state));
        // controller.set_text(2, 2, "Pos: " + std::to_string(motorPosition));
    }

    void lift() {
        if (lift_state == 2) {  
            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
                Motor::lb.move(127);
                pressCounter = 0;
                std::cout << "[Driver] Manual Move UP" << std::endl;
            }
            else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
                Motor::lb.move(-127);
                pressCounter++;
                std::cout << "[Driver] Manual Move DOWN" << std::endl;
                if (Motor::lb.get_position() < 50) lift_state = 0;
            }
            else {
                Motor::lb.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
                Motor::lb.brake();
                pressCounter = 0;
            }

            if (pressCounter > 40 || Motor::lb.get_position() < 50) lift_state = 0;
        }
        else autolift();

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2) || 
            controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            autonomousMode = false;
        }
    }
} // namespace LiftSix

namespace LiftSeven {
    int lift_state = 0, pressCounter = 0;
    bool autonomousMode = false;

    constexpr int RESET = 0, LOAD = 173, SCORE = 950, HANG = 600;
    std::vector<int> l_rot_target = {RESET, LOAD, SCORE};  // Relative positions

    double lift_kp_up = 1.4, lift_kp_down = 1.1, lift_kd = 0;
    static double lastError = 0, lastTime = pros::millis();

    void setState(int newTarget) {
        lift_state = newTarget; // should be something like lift target
        autonomousMode = true;
        std::cout << "[Auto] SetState called: Lift state = " << lift_state << std::endl;
    }

    void autolift() {
        if (!autonomousMode) {
            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
                if (lift_state < 2) lift_state++;
                controller.rumble(".");
                std::cout << "[Driver] L2 Pressed: Lift state = " << lift_state << std::endl;
            }
            else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
                if (lift_state > 0) lift_state--;
                std::cout << "[Driver] R2 Pressed: Lift state = " << lift_state << std::endl;
            }
        }

        double motorPosition = Motor::lb.get_position();
        double targetPosition = l_rot_target[lift_state];  // Relative target
        double error = targetPosition - motorPosition;
        double deltaTime = std::max((pros::millis() - lastTime) / 1000.0, 0.01);
        double derivative = (error - lastError) / deltaTime;
        double velocity = (error > 0) ? (lift_kp_up * error) : (lift_kp_down * error);
        velocity += lift_kd * derivative;

        if (std::fabs(error) < 10) {  
            Motor::lb.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            Motor::lb.brake();
        } else {
            Motor::lb.move(velocity);
        }

        if (lift_state == 0 && motorPosition < 15) {
            Motor::lb.tare_position();
            std::cout << "[Reset] Encoder reset at true bottom!" << std::endl;
        }

        lastError = error;
        lastTime = pros::millis();

        // Add back later
        // controller.set_text(0, 0, "State: " + std::to_string(lift_state));
        // controller.set_text(2, 2, "Pos: " + std::to_string(motorPosition));
    }

    void lift() {
        if (lift_state == 2) {  
            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
                Motor::lb.move(127);
                pressCounter = 0;
                std::cout << "[Driver] Manual Move UP" << std::endl;
            }
            else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
                Motor::lb.move(-127);
                pressCounter++;
                std::cout << "[Driver] Manual Move DOWN" << std::endl;
                if (Motor::lb.get_position() < 50) lift_state = 1; // Change to lift_state = 1;
            }
            else {
                Motor::lb.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
                Motor::lb.brake();
                pressCounter = 0;
            }

            if (pressCounter > 40 || Motor::lb.get_position() < 50) lift_state = 0;
        }
        else autolift();

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2) || 
            controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            autonomousMode = false;
        }
    }
} // namespace LiftSeven

// <------------------------------------------------------------- Color Sort ------------------------------------------------------------->
namespace Color {
    enum class colorVals { NONE, BLUE, RED };
    constexpr int DIST_VAL = 10;
    constexpr double rLow = 10.0, rHigh = 50.0, bLow = 164.0, bHigh = 213.5;
    inline bool isRed(double h) { return h > rLow && h < rHigh; }
    inline bool isBlue(double h) { return h > bLow && h < bHigh; }
    void colorSort(colorVals input) {
        colorVals lastColor = colorVals::NONE;
        while (1) {
            double hue = Sensor::o_colorSort.get_hue();
            double dist = Sensor::d_colorSort.get_distance();
            if (isRed(hue)) lastColor = colorVals::RED;
            else if (isBlue(hue)) lastColor = colorVals::BLUE;
            if (lastColor == input && dist < DIST_VAL) {
                Piston::sorter.set_value(true);
                pros::delay(350);
                lastColor = colorVals::NONE;
            } 
            else Piston::sorter.set_value(false);
            pros::delay(10);
        }
    }
} // namespace Color

// <------------------------------------------------------------- Tier Three ------------------------------------------------------------->
namespace Hang{
    const int UNWRAP_TIME = 750, TARGET_POS = 30;
    void reset(){ Misc::cdrift(127,127,UNWRAP_TIME); }
    void up(int iterE, int cd){ Misc::cdrift(-127,-127,iterE); pros::delay(cd); }
    void iter(int iterE, int cd, int times){ for(int i=0;i<times;i++){ up(iterE,cd); reset(); }}
    void move(int iterE, int cd){
        Piston::pto.set_value(true);
        Misc::cdrift(-15,-15,200); // Gear init
        iter(iterE, cd, 3);
    }
    void start(){
        Piston::pto.set_value(true);
        TaskHandler::colorSort = false;
        TaskHandler::lb = false;
        Motor::lb.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        Motor::lb.move(85);
        pros::delay(450);
        Motor::lb.move(0);
        // LiftSeven::setState(LiftSeven::HANG);
        for(int i = 1; i < 2; i++){
            do{
                leftMotors.move(-127);
                rightMotors.move(-127);
            }
            while(Sensor::hang.get_distance() > 20);
            reset();
            Motor::lb.move(85);
            pros::delay(450);
            Motor::lb.move(0);
            // LiftSeven::setState(LiftSeven::HANG);
        }   
    }
} // namespace Hang

// <-------------------------------------------------------------- Auto Routes ----------------------------------------------------------->
namespace Auton{
    int state = 0;
    namespace Test{
        void main() { 
            chassis.setPose(0, 0, 0);
            chassis.moveToPoint(0, 48, 10000, {.maxSpeed = 100});

            // chassis.turnToHeading(10, 3000);
            // pros::delay(1000);
            // chassis.turnToHeading(20, 3000);
            // pros::delay(1000);
            // chassis.turnToHeading(0, 3000);
            // pros::delay(1000);

            // chassis.turnToHeading(90, 3000);
            // pros::delay(1000);
            // chassis.turnToHeading(180, 3000);
            // pros::delay(1000);
            // // // chassis.turnToHeading(270, 3000);
            // chassis.turnToHeading(0, 3000);

            // Color::state = Color::sRed; 
            // LiftSeven::setState(LiftSeven::LOAD);
            // pros::delay(1000);
            // LiftSeven::setState(450);
            // pros::delay(1000);
            // LiftSeven::setState(LiftSeven::RESET);
        }
        void coords(){ 
            points.emplace_back(-24,24);
            points.emplace_back(-7,41);
            points.emplace_back(24,48);
            Misc::chain(points); // vec, angular timeout, lateral timeout
        }
        void linear(){
            Misc::linear(24,2000,{.forwards = true,.maxSpeed = 127,.minSpeed = 10,.earlyExitRange = 2});
        }
    } // namespace Test
    namespace Red{
        namespace Qual{
            void neg(){

            }
            void pos(){

            }
            void solo(){

            }
        } // namespace Qual
        namespace Elim{
            void neg(){

            }
            void pos(){

            }
        } // namespace Elim
    } // namespace Red
    namespace Blue{
        namespace Qual{
            void neg(){

            }
            void pos(){

            }
            void solo(){

            }
        } // namespace Qual
        namespace Elim{
            void neg(){

            }
            void pos(){

            }
        } // namespace Elim
    } // namespace Blue   
    namespace Skills{
        void main(){

        }
        void v1(){

        }
    } // namespace Skills
} // namespace Auton

// <-------------------------------------------------------------- Driver Code ----------------------------------------------------------->
namespace Driver{
    bool b_mogo = false, b_pto = false, b_grab = false;
    void joystick(){
        while(1){
            int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
            int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
            chassis.arcade(leftY, rightX);
            pros::delay(Misc::DELAY);
        }
    }
    void intake(){
        while(1){
            if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) { Motor::intakeT.move(127); }
            else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) { Motor::intakeT.move(-127); }
            else{ Motor::intakeT.set_brake_mode(pros::E_MOTOR_BRAKE_COAST); Motor::intakeT.set_brake_mode(pros::E_MOTOR_BRAKE_COAST); Motor::intakeT.brake(); }
            pros::delay(Misc::DELAY);
        }
    }
    void ladyBrown(){
        while(1){
            if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) { Motor::lb.move(127); }
            else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) { Motor::lb.move(-127); }
            else{ Motor::lb.set_brake_mode(pros::E_MOTOR_BRAKE_COAST); Motor::lb.brake(); }
            pros::delay(Misc::DELAY);
        }
    }
    void piston(){
        while(1){
            if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) { Misc::togglePiston(Piston::mogo, b_mogo); }
            if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) { Misc::togglePiston(Piston::pto, b_pto); } // Testing | Del 
            if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) { Misc::togglePiston(Piston::saberclamp, b_grab); }
            (controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) ? Piston::lightsaberL.set_value(true) : Piston::lightsaberL.set_value(false);
            // (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) ? Piston::lightsaberR.set_value(true) : Piston::lightsaberR.set_value(false);
            // (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) ? Piston::intake.set_value(true) : Piston::intake.set_value(false);
            if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) { Hang::start(); } 
            pros::delay(Misc::DELAY);
        }
    }
} // namespace Driver

namespace Screen {
    void update() {
        controller.clear();  
        pros::delay(500);
        controller.set_text(0, 0, "Run Time: " + std::to_string(pros::millis() / 1000) + "s");
        controller.set_text(1, 0, "Test Text 1");
        controller.set_text(2, 0, "Test Text 2");
        controller.set_text(3, 0, "Drive Left Temp: " + std::to_string(leftMotors.get_temperature()) + "C");
        controller.set_text(4, 0, "Drive Right Temp: " + std::to_string(rightMotors.get_temperature()) + "C");
        controller.set_text(5, 0, "Lift Motor Temp: " + std::to_string(Motor::lb.get_temperature()) + "C");
        controller.set_text(6, 0, "Intake Motor Temp: " + std::to_string(Motor::intakeT.get_temperature()) + "C");
        controller.set_text(7, 0, "Inertial Yaw: " + std::to_string(s_imu.get_rotation()));
        controller.set_text(8, 0, "Battery: " + std::to_string(pros::battery::get_capacity()) + "%");
        pros::delay(500);
    }
}

pros::Task screenTask([]{ while (true) { Screen::update(); }});

using AutonFunc = void(*)();
std::vector<std::pair<std::string, AutonFunc>> autonRoutines = {
    {"Default Auton", Auton::Test::main},
    {"Red Negative Quals", Auton::Red::Qual::neg},
    {"Blue Negative Quals", Auton::Blue::Qual::neg},
    {"Red Positive Quals", Auton::Red::Qual::pos},
    {"Blue Positive Quals", Auton::Blue::Qual::pos},
    {"Red Solo", Auton::Red::Qual::solo},
    {"Blue Solo", Auton::Blue::Qual::solo},
    {"Skills", Auton::Skills::main}
};

void autonSwitch() {
    while (1) {
        pros::delay(Misc::DELAY);
        if (Sensor::autonSwitch.get_new_press()) { Auton::state++; if (Auton::state >= autonRoutines.size()) Auton::state = 0; }
        pros::lcd::set_text(4, autonRoutines[Auton::state].first);
    }
}

Color::colorVals getColor(bool colorValV3) { return colorValV3 ? Color::colorVals::RED : Color::colorVals::BLUE; } // Sort ? Red : Blue

LV_IMG_DECLARE(tdbg);
LV_IMG_DECLARE(logo);

lv_obj_t * sbg = lv_img_create(lv_scr_act());
lv_obj_t * slogo = lv_img_create(lv_scr_act());

// <------------------------------------------------------------ Initialize --------------------------------------------------------------->
void initialize() {
    // lv_img_set_src(sbg, &tdbg);
	// lv_obj_set_pos(sbg,0,0);
	// lv_img_set_src(slogo, &logo);
	// lv_obj_set_pos(slogo,105,-15);
    // pros::Task t_Select(autonSelectSwitch);
    pros::lcd::initialize();
    chassis.setPose(0, 0, 0);
    chassis.calibrate(); 
    Motor::lb.set_zero_position(0.0);
    controller.clear();
    pros::Task screenTask([&]() {
        while (1) {
            // pros::lcd::print(3, "Pos: %d", Sensor::lbR.get_position());
            pros::lcd::print(3, "Pos: %f", Motor::lb.get_position());
            // pros::lcd::print(0, "X: %f", chassis.getPose().x);
            // pros::lcd::print(1, "Y: %f", chassis.getPose().y);
            // pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
            pros::delay(50);
        }
    });

    // pros::Task liftC([]{ while (1) { LiftSeven::lift(); pros::delay(Misc::DELAY); }});
    // pros::Task screenC([]{ while (1) { Screen::update(); pros::delay(100); }});
    Motor::intakeT.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    Sensor::o_colorSort.set_led_pwm(100);
    Sensor::o_colorSort.set_integration_time(10);
}
void disabled() {}
void competition_initialize() {}
ASSET(example_txt); // PP
// <------------------------------------------------------------- Autonomous ------------------------------------------------------------->
void autonomous() {
    // pros::Task sorterC([&]() { if(TaskHandler::colorSort) Color::colorSort(getColor(Color::state)); pros::delay(10);}); // back
    pros::Task sorterC([&]() { if(TaskHandler::colorSort) Color::colorSort(Color::colorVals::RED); pros::delay(10);});
    // Color::colorSort(Color::colorVals::RED);
    Auton::Test::main();
    pros::delay(10000000);
    (Auton::state < autonRoutines.size()) ? autonRoutines[Auton::state].second() : Auton::Test::main();
}

// <--------------------------------------------------------------- Driver --------------------------------------------------------------->
void opcontrol() {
    // Hang::start();
    // pros::delay(10000000);
    // pros::Task sorterC([&]() { if(TaskHandler::colorSort) Color::colorSort(getColor(Color::state)); pros::delay(10);}); // back
    pros::Task sorterC([&]() { if(TaskHandler::colorSort) Color::colorSort(Color::colorVals::RED); pros::delay(10);});
    pros::Task driverTask(Driver::joystick);
    pros::Task intakeTask(Driver::intake);
    pros::Task ladyBrownTask(Driver::ladyBrown);
    pros::Task pistonTask(Driver::piston);
	leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
	rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    // Motor::lb.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    while(1) {
        pros::delay(Misc::DELAY);
    }
}