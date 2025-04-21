// <--------------------------------------------------------------- Includes --------------------------------------------------------------->
#include <bits/stdc++.h>
#include <vector>
#include <functional>
#include <string>
#include "main.h"
#include "genesis/api.hpp"
#include "liblvgl/lvgl.h"
#include "liblvgl/llemu.hpp"
#include "brainScreenLVGL.h"
#include "config.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"

std::vector<std::pair<float, float>> points;
// Auton::state = 0;

namespace TaskHandler {
    bool colorSort = true;
    bool isShared = !colorSort;
    bool lbD = true;
    bool lbH = false;
    bool autoIntake = false;
} // namespace TaskHandler

// <------------------------------------------------------------ Miscellaneous ------------------------------------------------------------>
namespace Misc{
    constexpr int DELAY = 10;
    inline bool intakeR = false;
    pros::motor_brake_mode_e_t brakeState = pros::E_MOTOR_BRAKE_HOLD;
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
    void cdrift(float lV, float rV){
        leftMotors.move(lV);
        rightMotors.move(rV);
    }
    void cbrake(bool cst = true){
        (cst == true) ? (leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST), rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST)) : (leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE), rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE));
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
namespace Lift {
    int lift_index = 0; // Index for cycling through presets
    int lift_target_position = 0; // Absolute target position
    bool autonomousMode = false;

    constexpr int RESET = 0, LOAD = 160, SCORE = 1050, HANG = 600; // (216)
    std::vector<int> l_rot_target = {RESET, LOAD, SCORE};  // Preset lift positions

    double lift_kp_up = 1.0, lift_kp_down = 1.0, lift_kd = 0.0, dM = 1.0;
    static double lastError = 0, lastTime = pros::millis();

    void setState(int newTarget) {
        lift_target_position = newTarget;

        // // Check if the new target is in the list
        // auto it = std::find(l_rot_target.begin(), l_rot_target.end(), newTarget);
        // if (it != l_rot_target.end()) {
        //     lift_index = std::distance(l_rot_target.begin(), it);
        // } else {
        //     lift_index = -1; // Allow custom targets
        // }

        // Always trigger intake reversal if going up from LOAD, regardless of index
        if (lift_target_position > LOAD && Motor::lbL.get_position() <= LOAD) {
            pros::Task([] {
                Misc::intakeR = true;
                Motor::intakeT.move(-25);
                pros::delay(250);
                Motor::intakeT.move(0);
                Misc::intakeR = false;
                std::cout << "[Auto] Intake reversed briefly on lift from LOAD\n";
            });
        }

        autonomousMode = true;
        std::cout << "[Auto] SetState called: Lift target = " << lift_target_position << std::endl;
    }

    void autolift() {
        static bool locked = false;
        static int prev_target_position = lift_target_position;

        // Allow manual increment/decrement through controller
        if (!autonomousMode) {
            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
                if (lift_index < (int)l_rot_target.size() - 1) {
                    int new_index = lift_index + 1;
                    int new_target = l_rot_target[new_index];

                    // Check for transition from LOAD to higher
                    if (lift_target_position == LOAD && new_target > LOAD) {
                        pros::Task([] {
                            Misc::intakeR = true;
                            Motor::intakeT.move(-25);
                            pros::delay(250);
                            Motor::intakeT.move(0);
                            Misc::intakeR = false;
                            std::cout << "[Driver] Intake reversed briefly on lift from LOAD\n";
                        });
                    }

                    lift_index = new_index;
                    lift_target_position = new_target;
                    controller.rumble(".");
                    std::cout << "[Driver] L2 Pressed: Index = " << lift_index 
                            << ", Target = " << lift_target_position << std::endl;
                }
            } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
                if (lift_index > 0) {
                    lift_index--;
                    lift_target_position = l_rot_target[lift_index];
                    std::cout << "[Driver] R2 Pressed: Index = " << lift_index 
                            << ", Target = " << lift_target_position << std::endl;
                }
            }
        }

        double motorPosition = Motor::lbL.get_position();
        double error = lift_target_position - motorPosition;
        double deltaTime = std::max((pros::millis() - lastTime) / 1000.0, 0.01); // seconds
        double derivative = (error - lastError) / deltaTime;
        // Basic PID velocity
        double velocity = (error > 0) ? (lift_kp_up * error) : ((lift_kp_down * error) * dM);

        // If moving from RESET to LOAD, cap upward speed to avoid overshoot
        if (lift_target_position == LOAD && Motor::lbL.get_position() < LOAD) {
            velocity = std::clamp(velocity, -50.0, 50.0);  // upward cap = 60
        }

        velocity += lift_kd * derivative;

        if (lift_target_position == RESET && std::fabs(error) < 5) {
            Motor::lbL.move(0);
            Motor::lbR.move(0);
            locked = true;
            return;
        }


        else {
            locked = false;
            Motor::lbL.move(velocity);
            Motor::lbR.move(velocity);
        }

        lastError = error;
        lastTime = pros::millis();
    }

    void lift() {
        if (lift_target_position == SCORE) {  // Manual override only at SCORE height
            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
                Motor::lbL.move(127);
                Motor::lbR.move(127);
                std::cout << "[Driver] Manual Move UP" << std::endl;
            } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
                Motor::lbL.move(-127);
                Motor::lbR.move(-127);
                std::cout << "[Driver] Manual Move DOWN" << std::endl;

                if (Motor::lbL.get_position() < 50) {
                    lift_index = 1;
                    lift_target_position = l_rot_target[lift_index];
                }
            } else {
                Motor::lbL.set_brake_mode(Misc::brakeState);
                Motor::lbR.set_brake_mode(Misc::brakeState);
                Motor::lbL.brake();
                Motor::lbR.brake();
            }

        } else {
            autolift();
        }

        // Any manual control disables auto mode
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2) || 
            controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            autonomousMode = false;
        }
    }
} // namespace Lift

bool info = false;
// <------------------------------------------------------------- Color Sort ------------------------------------------------------------->
namespace Color {
    enum class colorVals { NONE, BLUE, RED };
    colorVals state = colorVals::NONE;
    bool isDone = false;
    constexpr double rLow = 5.0, rHigh = 20.0, bLow = 180.0, bHigh = 230.0, minProx= 195; // Values for colorSort
    constexpr double rLow1 = 9.0, rHigh1 = 36.0, bLow1 = 175.0, bHigh1 = 230.0, minProx1 = 150; // Values for ring store on intake
    inline bool isRed(double h, double low, double max) { return h > low && h < max; }
    inline bool isBlue(double h, double low, double max) { return h > low && h < max; }
    inline bool withinProx(int input, double max) { return (input > max); }
    colorVals colorConvertor(colorVals input) { return (input == colorVals::BLUE) ? colorVals::RED : colorVals::BLUE; }
    void colorSort(colorVals input) {
        colorVals lastColor = colorVals::NONE;
        while (1) {
            if(isRed(Sensor::o_colorSort.get_hue(),rLow,rHigh) && withinProx(Sensor::o_colorSort.get_proximity(),minProx)) { lastColor = colorVals::RED; } 
            else if(isBlue(Sensor::o_colorSort.get_hue(),bLow,bHigh) && withinProx(Sensor::o_colorSort.get_proximity(),minProx)) { lastColor = colorVals::BLUE; }
            if(input == lastColor){
                pros::delay(65);
                Motor::intakeT.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
                Motor::intakeT.brake();
                // Motor::intakeT.move(-5);
                pros::delay(150);
                Motor::intakeT.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
                Motor::intakeT.move(127);
                // Motor::intakeT.move(TaskHandler::sharedSpeed);
                lastColor = colorVals::NONE;
            } 
            pros::delay(Misc::DELAY);
        }
    }

    void toPos(Color::colorVals input){ // Color to be hovered
        colorVals lastColor = colorVals::NONE;
            Sensor::o_colorSort.set_led_pwm(100);
            if (isRed(Sensor::o_colorSort.get_hue(),rLow1,rHigh1)) lastColor = colorVals::RED;
            else if (isBlue(Sensor::o_colorSort.get_hue(),bLow1,bHigh1)) lastColor = colorVals::BLUE;
            if(input == lastColor){
                if(withinProx(Sensor::o_colorSort.get_proximity(),minProx1)) {
                    // TaskHandler::sharedSpeed = 0;
                    Motor::intakeT.move(-5);
                    pros::delay(150);
                    Motor::intakeT.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
                    Motor::intakeT.brake();
                }
            }
            // TaskHandler::sharedSpeed = 127;
            pros::delay(Misc::DELAY);
    }
} // namespace Color

// <------------------------------------------------------------- Tier Three ------------------------------------------------------------->
namespace Hang{
    constexpr int UNWRAP_TIME = 750, TARGET = 500, DIST_SENSED = 15;
    double currPos = 0, kP = 0.35;

    void pull(){
        leftMotors.move(-127);
        rightMotors.move(-127);
        do{
            pros::delay(5);
        }
        while(!(Sensor::hang.get_distance()<15));
        leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
        rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
        leftMotors.brake();
        rightMotors.brake();
    }
    
    void release(){
        leftMotors.tare_position_all();
        rightMotors.tare_position_all();
        TaskHandler::lbD = true;
        Lift::setState(900); 
        do{
            currPos = (leftMotors.get_position()+rightMotors.get_position())/2;
            double velocity = (TARGET - currPos)*kP;
            leftMotors.move(velocity);
            rightMotors.move(velocity);
            pros::delay(Misc::DELAY);
        }
        while(!(currPos > TARGET));
        TaskHandler::lbD = false;
        Motor::lbL.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        Motor::lbR.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    }

    void init(){
        TaskHandler::colorSort = false;
        Piston::pto.set_value(true);
        Piston::release.set_value(true);
    }
} // namespace Hang

// <-------------------------------------------------------------- Auto Routes ----------------------------------------------------------->
namespace Auton{
    int state = 0;
    namespace Test{
        void main() { 
            // pros::Task colorSort(Color::colorSort())
            Color::state = Color::colorVals::BLUE;
            // Motor::intakeT.move(127);
            // Misc::cdrift(30,30,900);

            // Lift::setState(Lift::LOAD);
            // pros::delay(500);
            // Motor::intakeT.move(127);
            // pros::delay(1000);
            // Lift::setState(1500);
            // pros::delay(2000);
            // Lift::setState(0);
            // pros::delay(1000);
            // Motor::intakeT.move(127);


            // // Motor::lbL.set_zero_position(193); Motor::lbR.set_zero_position(193);
            // Lift::setState(1500);
            // pros::delay(2000);
            // Lift::setState(0);
            // pros::delay(1000);
            // Motor::intakeT.move(127);

            // Lift::setState(Lift::LOAD);
            // pros::delay(1000);
            Lift::setState(1050);
            pros::delay(1000);
            Lift::setState(Lift::RESET);
            pros::delay(1000);

            // chassis.setPose(0, 0, 0);
            // chassis.moveToPoint(0, 48, 10000, {.maxSpeed = 100});

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
                Color::state = Color::colorVals::BLUE;
                chassis.setPose(-53,9,-114);
                Lift::setState(1700);
                pros::delay(1000);
                chassis.moveToPoint(-22, 25, 2000,{.forwards = false,.maxSpeed = 75});
                Piston::mogo.set_value(true);
                pros::delay(100);
                Motor::intakeT.move(127);
                chassis.turnToPoint(-10.5, 38, 650, {.forwards = true,.maxSpeed=127,.minSpeed=0});
                chassis.moveToPoint(-10.5, 38, 1500, {.forwards = true,.maxSpeed=127,.minSpeed = 10,.earlyExitRange=1});
                chassis.waitUntilDone();
                pros::delay(150);
                
                leftMotors.move(-60);
                rightMotors.move(-60);
                pros::delay(250);
                leftMotors.brake();
                rightMotors.brake();
                chassis.turnToPoint(-24, 48, 750, {.forwards = true,.maxSpeed=60,.minSpeed=0});
                chassis.moveToPoint(-24, 48, 1200, {.forwards = true,.maxSpeed=127,.minSpeed = 10,.earlyExitRange=1});
                chassis.turnToPoint(-48, 48, 350, {.forwards = true,.maxSpeed=127,.minSpeed=0});
                chassis.moveToPoint(-48, 48, 1200, {.forwards = true,.maxSpeed=127,.minSpeed = 10,.earlyExitRange=1});
                chassis.turnToPoint(-70, 70, 1200, {.forwards = true,.maxSpeed=127,.minSpeed=0});
                Misc::cdrift(60,60,1600);
                chassis.moveToPoint(-24, 15, 1200, {.forwards = false,.maxSpeed=127,.minSpeed = 10,.earlyExitRange=1});
                chassis.turnToPoint(-9, 2, 1200, {.forwards = true,.maxSpeed=127,.minSpeed=0});
                Lift::setState(1250);
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
    bool b_mogo = false, b_pto = false, b_grab = false, b_doink = false,b_hang = false;
    int saberC = 0;
    void joystick(){
        while(1){
            int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
            int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
            chassis.arcade(leftY, rightX);
            pros::delay(Misc::DELAY);
        }
    }
    void wall(){
        Lift::setState(1400);
        Motor::lbL.move(127);
        Motor::lbR.move(127);
        Misc::cdrift(-40,-40,600,false);
    }
    void intake(){
        while(1){
            if(!Misc::intakeR){
                if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) { Motor::intakeT.move(127); }
                else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) { Motor::intakeT.move(-127); }
                else{ Motor::intakeT.set_brake_mode(pros::E_MOTOR_BRAKE_COAST); Motor::intakeT.set_brake_mode(pros::E_MOTOR_BRAKE_COAST); Motor::intakeT.brake(); }
            }
            pros::delay(Misc::DELAY);
        }
    }
    // void ladyBrown(){
    //     while(1){
    //         if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) { Motor::lb.move(127); }
    //         else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) { Motor::lb.move(-127); }
    //         else{ Motor::lb.set_brake_mode(pros::E_MOTOR_BRAKE_COAST); Motor::lb.brake(); }
    //         pros::delay(Misc::DELAY);
    //     }
    // }
    void piston(){
        while(1){
            if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) { 
                if(saberC == 0) { Piston::lightsaberL.set_value(true); saberC++; }
                else if(saberC == 1) { Piston::saberclamp.set_value(true); saberC++; }
                else if(saberC == 2) { Piston::saberclamp.set_value(false); Piston::lightsaberL.set_value(false); saberC = 0; }
            }
            if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) { Misc::togglePiston(Piston::mogo, b_mogo); }
            // if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) { Misc::togglePiston(Piston::saberclamp, b_grab); }
            // (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) ? Piston::lightsaberL.set_value(true) : Piston::lightsaberL.set_value(false);
            pros::delay(Misc::DELAY);
        }
    }
    void hang(){
        bool initT = false, isUp = false;
        while(1){
            if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN) && initT == false) { Misc::togglePiston(Piston::release, b_hang); initT = true; } 
            if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) { 
                Misc::togglePiston(Piston::pto, b_pto); 
                Misc::brakeState = (Misc::brakeState == pros::E_MOTOR_BRAKE_HOLD) ? pros::E_MOTOR_BRAKE_COAST : pros::E_MOTOR_BRAKE_HOLD;
            }
            // else if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN) && initT == true && isUp == false) { Hang::pull(); isUp =!isUp; } 
            // else if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN) && initT == true && isUp == true) { Hang::release(); isUp =!isUp; } 
            pros::delay(Misc::DELAY);
        }
    }
} // namespace Driver
// <-------------------------------------------------------------- Auton ----------------------------------------------------------->
namespace Screen {
    void update() {
        controller.clear();  
        pros::delay(500);
        // controller.set_text(0, 0, "Sensed?: " + std::to_string(info) + "Speed" + std::to_string(Color::lastSpeed));
        // controller.set_text(0, 0, "1: " + std::to_string(Sensor::d_colorSort.get_distance()));
        controller.set_text(0, 0, "Pos: " + std::to_string(Motor::lbL.get_position()));
        // controller.set_text(0, 0, "Run Time: " + std::to_string(pros::millis() / 1000) + "s");
        controller.set_text(1, 0, "Test Text 1");
        controller.set_text(2, 0, "Test Text 2");
        // controller.set_text(3, 0, "Drive Left Temp: " + std::to_string(leftMotors.get_temperature()) + "C");
        // controller.set_text(4, 0, "Drive Right Temp: " + std::to_string(rightMotors.get_temperature()) + "C");
        // controller.set_text(5, 0, "Lift Motor Temp: " + std::to_string((Motor::lbL.get_temperature()+Motor::lbR.get_temperature())/2) + "C");
        // controller.set_text(6, 0, "Intake Motor Temp: " + std::to_string(Motor::intakeT.get_temperature()) + "C");
        // controller.set_text(7, 0, "Inertial Yaw: " + std::to_string(s_imu.get_rotation()));
        // controller.set_text(8, 0, "Battery: " + std::to_string(pros::battery::get_capacity()) + "%");
        pros::delay(500);
    }
}

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
    // pros::Task t_Select(autonSelectSwitch);
    pros::lcd::initialize();
    chassis.setPose(0, 0, 0);
    chassis.calibrate(); 
    Motor::lbL.set_zero_position(0.0);
    Motor::lbR.set_zero_position(0.0);
    Sensor::o_colorSort.set_led_pwm(100);
    Sensor::o_colorSort.set_integration_time(10);
    controller.clear();
    // lv_img_set_src(sbg, &tdbg);
	// lv_obj_set_pos(sbg,0,0);
	// lv_img_set_src(slogo, &logo);
	// lv_obj_set_pos(slogo,105,-15);
    pros::Task screenTask([&]() {
        while (1) {
            // pros::lcd::print(3, "Pos: %d", Sensor::lbR.get_position());
            // pros::lcd::print(3, "Pos: %f", Motor::lb.get_position());
            pros::lcd::print(0, "X: %f", chassis.getPose().x);
            pros::lcd::print(1, "Y: %f", chassis.getPose().y);
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
            pros::delay(50);
        }
    });

    pros::Task liftC([]{ while (1) { if(TaskHandler::lbD) Lift::lift(); pros::delay(Misc::DELAY); }});
    pros::Task screenC([]{ while (1) { Screen::update(); pros::delay(100); }});
    Motor::intakeT.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}
void disabled() {}
void competition_initialize() {}
ASSET(example_txt); // PP

// <------------------------------------------------------------- Autonomous ------------------------------------------------------------->
void autonomous() {
    pros::Task sorterC([&](){ while(1) { Color::colorSort(Color::state); pros::delay(Misc::DELAY); }});
    pros::Task toPosC([&](){ while(1) { if(TaskHandler::autoIntake) Color::toPos(Color::colorConvertor(Color::state)); pros::delay(Misc::DELAY); }});
    chassis.setPose(0,0,0);
    chassis.moveToPose(0, 23.8*2, 0, 4000);
    controller.set_text(1, 0, "Test Text 1");    
    //Auton::Test::main(); 
    pros::delay(10000000);
    (Auton::state < autonRoutines.size()) ? autonRoutines[Auton::state].second() : Auton::Test::main();
}

// <--------------------------------------------------------------- Driver --------------------------------------------------------------->
void opcontrol() {
    // pros::Task sorterC([&](){ while(1) { if(TaskHandler::colorSort) Color::colorSort(Color::colorVals::RED);  pros::delay(Misc::DELAY);}});
    // pros::Task sorterC(Color::colorSort(Color::colorVals::RED));
    
    pros::Task intakeTask(Driver::intake);
    pros::Task driverTask(Driver::joystick);
    pros::Task pistonTask(Driver::piston);
    pros::Task hangTask(Driver::hang);
	leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
	rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    Motor::lbL.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    Motor::lbR.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    while(1) {
        // if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) { Lift::setState(1160); }
        // if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) { Driver::wall(); }
        pros::delay(Misc::DELAY);
    }
}