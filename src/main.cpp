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
    int sharedSpeed = 127;
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
    enum LiftMode { RESET, LOAD, SCORE };
    LiftMode lift_mode = RESET;

    int lift_target_position = 0;
    bool autonomousMode = false;
    bool manualOverride = false;

    constexpr int RESET_POS = 0, LOAD_POS = 160, SCORE_POS = 1050;
    double lift_kp_up = 1.0, lift_kp_down = 1.0, lift_kd = 0.0, dM = 1.0;

    static double lastError = 0;
    static double lastTime = pros::millis();

    void setState(int newTarget) {
        lift_target_position = newTarget;
        autonomousMode = true;
        manualOverride = false;
        std::cout << "[Auto] SetState: Target = " << lift_target_position << std::endl;
    }

    void updatePID() {
        double pos = Motor::lbL.get_position();
        double error = lift_target_position - pos;
        double dt = std::max((pros::millis() - lastTime) / 1000.0, 0.01);
        double derivative = (error - lastError) / dt;

        double velocity = (error > 0)
            ? (lift_kp_up * error)
            : (lift_kp_down * error) * dM;

        velocity += lift_kd * derivative;
        velocity = std::clamp(velocity, -80.0, 80.0);

        if (lift_target_position == RESET_POS && std::fabs(error) < 5) {
            Motor::lbL.move(0);
            Motor::lbR.move(0);
        } else {
            Motor::lbL.move(velocity);
            Motor::lbR.move(velocity);
        }

        lastError = error;
        lastTime = pros::millis();
    }

    // void driverControl() {
    //     static uint32_t r2HoldStart = 0;

    //     bool L2_new = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2);
    //     bool L2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
    //     bool R2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);

    //     // ---------------------------------------
    //     // Priority 1: Handle L2 press transitions
    //     // ---------------------------------------
    //     if (L2_new) {
    //         if (lift_mode == RESET) {
    //             lift_mode = LOAD;
    //             lift_target_position = LOAD_POS;
    //             manualOverride = false;
    //             autonomousMode = false;
    //             std::cout << "[Driver] L2 press → LOAD\n";
    //         } else if (lift_mode == LOAD) {
    //             lift_mode = SCORE;
    //             manualOverride = true;
    //             autonomousMode = false;
    //             std::cout << "[Driver] L2 press → SCORE (manual)\n";
    //         }
    //     }

    //     // ------------------------------------------------------
    //     // Priority 2: R2 pressed goes back to RESET from any mode
    //     // ------------------------------------------------------
    //     if ((lift_mode == LOAD || lift_mode == SCORE) && R2) {
    //         lift_mode = RESET;
    //         lift_target_position = RESET_POS;
    //         manualOverride = false;
    //         autonomousMode = false;
    //         std::cout << "[Driver] R2 press → RESET\n";
    //     }

    //     // ---------------------------------------------------------
    //     // Priority 3: R2 held in RESET triggers lift reset command
    //     // ---------------------------------------------------------
    //     if (lift_mode == RESET && R2) {
    //         if (r2HoldStart == 0) r2HoldStart = pros::millis();
    //         if (pros::millis() - r2HoldStart > 500) {
    //             std::cout << "[Driver] R2 held → Resetting lift\n";
    //             lift_target_position = RESET_POS;
    //             r2HoldStart = 0;
    //         }
    //     } else {
    //         r2HoldStart = 0;
    //     }

    //     // --------------------------------------
    //     // Priority 4: Manual control in SCORE mode
    //     // --------------------------------------
    //     if (lift_mode == SCORE && manualOverride) {
    //         if (L2) {
    //             Motor::lbL.move(127);
    //             Motor::lbR.move(127);
    //             std::cout << "[Driver] Manual Up\n";
    //         } else if (R2) {
    //             Motor::lbL.move(-127);
    //             Motor::lbR.move(-127);
    //             std::cout << "[Driver] Manual Down\n";
    //         } else {
    //             Motor::lbL.set_brake_mode(Misc::brakeState);
    //             Motor::lbR.set_brake_mode(Misc::brakeState);
    //             Motor::lbL.brake();
    //             Motor::lbR.brake();
    //         }
    //         return;
    //     }

    //     // --------------------------------
    //     // Priority 5: PID fallback control
    //     // --------------------------------
    //     updatePID();
    // }

    void driverControl() {
        static uint32_t r2HoldStart = 0;

        bool L2_new = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2);
        bool L2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
        bool R2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);

        // ---------------------------------------
        // Priority 1: Handle L2 press transitions
        // ---------------------------------------
        if (L2_new) {
            if (lift_mode == RESET) {
                lift_mode = LOAD;
                lift_target_position = LOAD_POS;
                manualOverride = false;
                autonomousMode = false;
                std::cout << "[Driver] L2 press → LOAD\n";
            } else if (lift_mode == LOAD) {
                lift_mode = SCORE;
                manualOverride = true;
                autonomousMode = false;
                std::cout << "[Driver] L2 press → SCORE (manual)\n";
            }
        }

        // --------------------------------------------
        // Priority 2: R2 press from LOAD → RESET only
        // --------------------------------------------
        if (lift_mode == LOAD && R2) {
            lift_mode = RESET;
            lift_target_position = RESET_POS;
            manualOverride = false;
            autonomousMode = false;
            std::cout << "[Driver] R2 press → RESET\n";
        }

        // ---------------------------------------------------------
        // Priority 3: R2 held in RESET triggers lift reset command
        // ---------------------------------------------------------
        if (lift_mode == RESET && R2) {
            if (r2HoldStart == 0) r2HoldStart = pros::millis();
            if (pros::millis() - r2HoldStart > 500) {
                std::cout << "[Driver] R2 held → Resetting lift\n";
                lift_target_position = RESET_POS;
                r2HoldStart = 0;
            }
        } else {
            r2HoldStart = 0;
        }

        // --------------------------------------
        // Priority 4: Manual control in SCORE mode
        // --------------------------------------
        if (lift_mode == SCORE && manualOverride) {
            if (L2) {
                Motor::lbL.move(127);
                Motor::lbR.move(127);
                std::cout << "[Driver] Manual Up\n";
            } else if (R2) {
                Motor::lbL.move(-127);  // Optional: use lower speed like -100 for safety
                Motor::lbR.move(-127);
                std::cout << "[Driver] Manual Down\n";
            } else {
                Motor::lbL.set_brake_mode(Misc::brakeState);
                Motor::lbR.set_brake_mode(Misc::brakeState);
                Motor::lbL.brake();
                Motor::lbR.brake();
            }
            return;
        }

        // --------------------------------
        // Priority 5: PID fallback control
        // --------------------------------
        updatePID();
    }



    void lift() {
        if (autonomousMode) {
            updatePID();
        } else {
            driverControl();
        }

        // Cancel autonomous if manual input occurs
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
                pros::delay(86);
                Motor::intake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
                Motor::intake.brake();
                Motor::intake.move(-5);
                pros::delay(175);
                Motor::intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
                // Motor::intake.move(127);
                Motor::intake.move(TaskHandler::sharedSpeed);
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
                    TaskHandler::sharedSpeed = 0;
                    Motor::intake.move(-5);
                    pros::delay(150);
                    Motor::intake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
                    Motor::intake.brake();
                }
            }
            TaskHandler::sharedSpeed = 127;
            pros::delay(Misc::DELAY);
    }
} // namespace Color

// <------------------------------------------------------------- Tier Three ------------------------------------------------------------->
namespace Hang{
    constexpr int UNWRAP_TIME = 750, TARGET = 4200, DIST_SENSED = 15;
    double currPos = 0, kP = 0.35;
    int timer = 0;

    void pull(){
        leftMotors.move(-127);
        rightMotors.move(-127);
        do{
            pros::delay(5);
        }
        while(!(Sensor::hang.get_distance()<15));
        leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
        rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
        leftMotors.brake();
        rightMotors.brake();
    }
    
    void release(){
        leftMotors.tare_position_all();
        rightMotors.tare_position_all();
        TaskHandler::lbD = true;
        Lift::setState(900);
        do{
            if(timer > 3000) break;
            currPos = leftMotors.get_position();
            double velocity = (TARGET - currPos)*kP;
            leftMotors.move(velocity);
            rightMotors.move(velocity);
            timer+=Misc::DELAY;
            pros::delay(Misc::DELAY);
        }
        while(currPos < TARGET);
        Motor::lbL.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        Motor::lbR.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        Lift::setState(400);
        // do{
        //     Lift::setState(400);
        //     pros::delay(Misc::DELAY);
        // }
        // while(Motor::lbL.get_position() < 395);

        // TaskHandler::lbD = false;
        // Motor::lbL.set_zero_position(900.0);
        // Motor::lbR.set_zero_position(900.0);
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
            // Motor::intake.move(127);
            // Misc::cdrift(30,30,900);

            // Lift::setState(Lift::LOAD);
            // pros::delay(500);
            // Motor::intake.move(127);
            // pros::delay(1000);
            // Lift::setState(1500);
            // pros::delay(2000);
            // Lift::setState(0);
            // pros::delay(1000);
            // Motor::intake.move(127);


            // // Motor::lbL.set_zero_position(193); Motor::lbR.set_zero_position(193);
            // Lift::setState(1500);
            // pros::delay(2000);
            // Lift::setState(0);
            // pros::delay(1000);
            // Motor::intake.move(127);

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
                Motor::intake.move(127);
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
                if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) { Motor::intake.move(127); }
                else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) { Motor::intake.move(-127); }
                else{ Motor::intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST); Motor::intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST); Motor::intake.brake(); }
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
            if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) { Hang::release(); }
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
        // controller.set_text(0, 0, "Pos: " + std::to_string(Motor::lbL.get_position()));
        controller.set_text(0, 0, "Dist: " + std::to_string(leftMotors.get_position()));
        // controller.set_text(0, 0, "Run Time: " + std::to_string(pros::millis() / 1000) + "s");
        controller.set_text(1, 0, "Test Text 1");
        controller.set_text(2, 0, "Test Text 2");
        // controller.set_text(3, 0, "Drive Left Temp: " + std::to_string(leftMotors.get_temperature()) + "C");
        // controller.set_text(4, 0, "Drive Right Temp: " + std::to_string(rightMotors.get_temperature()) + "C");
        // controller.set_text(5, 0, "Lift Motor Temp: " + std::to_string((Motor::lbL.get_temperature()+Motor::lbR.get_temperature())/2) + "C");
        // controller.set_text(6, 0, "Intake Motor Temp: " + std::to_string(Motor::intake.get_temperature()) + "C");
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
    Motor::intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
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
}
void disabled() {}
void competition_initialize() {}
ASSET(example_txt); // PP

// <------------------------------------------------------------- Autonomous ------------------------------------------------------------->
void autonomous() {
    // Piston::mogo.set_value(true);
    // pros::Task sorterC([&](){ while(1) { Color::colorSort(Color::state); pros::delay(Misc::DELAY); }});
    // pros::Task toPosC([&](){ while(1) { if(TaskHandler::autoIntake) Color::toPos(Color::colorConvertor(Color::state)); pros::delay(Misc::DELAY); }});
    // Lift::setState(900);
    // pros::delay(1000);
    // Lift::setState(400);

    // Color::state = Color::colorVals::RED;
    // Motor::intake.move(127);
    // TaskHandler::autoIntake = true;
    pros::lcd::print(5, "hellooo");
    chassis.setPose(0,0,0);
    chassis.moveToPoint(0, 23.8, 4000, {.maxSpeed = 20});
    chassis.brake();
    // chassis.turnToHeading(90,10000);
    // controller.set_text(1, 0, "Test Text 1"); 
    // //Auton::Test::main(); 
    // Lift::setState(Lift::LOAD_POS);
    // pros::delay(1000);
    // Lift::setState(900);
    // pros::delay(1000);
    // Lift::setState(Lift::RESET_POS);
    pros::delay(10000000);
    (Auton::state < autonRoutines.size()) ? autonRoutines[Auton::state].second() : Auton::Test::main();
}

// <--------------------------------------------------------------- Driver --------------------------------------------------------------->
void opcontrol() {
    // pros::Task sorterC([&](){ while(1) { if(TaskHandler::colorSort) Color::colorSort(Color::colorVals::RED);  pros::delay(Misc::DELAY);}});
    // pros::Task sorterC(Color::colorSort(Color::colorVals::RED));
    
    pros::Task intakeask(Driver::intake);
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