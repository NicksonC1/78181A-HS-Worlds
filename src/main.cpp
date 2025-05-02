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

namespace offset {
    double yAxis = -1.07;
}

namespace TaskHandler {
    bool antiJam = false;
    bool autonSelect = true;
    bool colorSort = true;
    bool isShared = !colorSort;
    bool lbD = true;
    bool lbH = false;
    bool autoIntake = false;
    int sharedSpeed = 127;
    bool isDriver = true;
    bool dIntake = true;
} // namespace TaskHandler

// <------------------------------------------------------------ Miscellaneous ------------------------------------------------------------>
namespace Misc{
    constexpr int DELAY = 10;
    constexpr double X = -7.0;
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

    constexpr int RESET_POS = 0, SCORE_POS = 1050, LOAD_DIST_TARGET = 128; 
    double lift_kp_up_rot = 0.8, lift_kp_down_rot = 0.8, lift_kd_rot = 0.0;
    double lift_kp_up_dist = 0.3, lift_kp_down_dist = 0.04, lift_kd_dist = 0.0;
    
    double dM = 1.0;

    static double lastError = 0;
    static double lastTime = pros::millis();


    void setState(int newTarget) {
        lift_target_position = newTarget;
        autonomousMode = true;
        manualOverride = false;

        if (newTarget >= 40 && newTarget <= 70) {
            lift_mode = LOAD;
        } else {
            lift_mode = RESET;
        }
    }



    double getLiftPosition() {
        if (lift_mode == LOAD) return Sensor::lbD.get();
        else return Motor::lbR.get_position();
    }

    void updatePID() {
        double pos = getLiftPosition();
        double target;
        double kp_up, kp_down, kd;

        if (lift_mode == LOAD) {
            target = LOAD_DIST_TARGET;
            kp_up = lift_kp_up_dist;
            kp_down = lift_kp_down_dist;
            kd = lift_kd_dist;
        } else {
            target = lift_target_position;
            kp_up = lift_kp_up_rot;
            kp_down = lift_kp_down_rot;
            kd = lift_kd_rot;
        }

        double error = target - pos;
        double dt = std::max((pros::millis() - lastTime) / 1000.0, 0.01);
        double derivative = (error - lastError) / dt;

        double velocity = (error > 0)
            ? (kp_up * error)
            : (kp_down * error) * dM;

        velocity += kd * derivative;
        velocity = std::clamp(velocity, -80.0, 80.0);

        if (lift_mode != LOAD && target == RESET_POS && std::fabs(error) < 5) {
            Motor::lbL.move(0);
            Motor::lbR.move(0);
        } else {
            Motor::lbL.move(velocity);
            Motor::lbR.move(velocity);
        }

        lastError = error;
        lastTime = pros::millis();
    }

    void driverControl() {
        static uint32_t r2HoldStart = 0;

        bool L2_new = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2);
        bool L2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
        bool R2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);

        if (L2_new) {
            if (lift_mode == RESET) {
                lift_mode = LOAD;
                manualOverride = false;
                autonomousMode = false;
            } else if (lift_mode == LOAD) {
                lift_mode = SCORE;
                manualOverride = true;
                autonomousMode = false;
            }
        }

        // Go from LOAD → RESET with R2
        if (lift_mode == LOAD && R2) {
            lift_mode = RESET;
            lift_target_position = RESET_POS;
            manualOverride = false;
            autonomousMode = false;
        }

        if (lift_mode == SCORE) {
            if (R2) {
                if (r2HoldStart == 0) r2HoldStart = pros::millis();
                uint32_t heldTime = pros::millis() - r2HoldStart;

                if (heldTime > 500) {
                    lift_mode = RESET;
                    lift_target_position = RESET_POS;
                    manualOverride = false;
                    autonomousMode = false;
                    r2HoldStart = 0;
                }
            } else if (r2HoldStart != 0) {
                lift_mode = LOAD;
                manualOverride = false;
                autonomousMode = false;
                r2HoldStart = 0;
            }
        }else if (lift_mode == RESET && R2) {
            if (r2HoldStart == 0) r2HoldStart = pros::millis();
            if (pros::millis() - r2HoldStart > 500) {
                lift_target_position = RESET_POS;
                r2HoldStart = 0;
            }
        } else {
            r2HoldStart = 0;
        }

        if (lift_mode == SCORE && manualOverride) {
            if (L2) {
                Motor::lbL.move(127);
                Motor::lbR.move(127);
            } else if (R2) {
                Motor::lbL.move(-127);
                Motor::lbR.move(-127);
            } else {
                Motor::lbL.set_brake_mode(Misc::brakeState);
                Motor::lbR.set_brake_mode(Misc::brakeState);
                Motor::lbL.brake();
                Motor::lbR.brake();
            }
            return;
        }
        updatePID();
    }

    void lift() {
        if (autonomousMode) updatePID();
        else driverControl();

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2) ||
            controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            autonomousMode = false;
        }
    }
} // namespace Lift

//<------------------------------------------------------------- Anti Jam ------------------------------------------------------------->
namespace Jam{
    int counter = 0;
    bool stuck = false;
    void antiJam(){
        if(TaskHandler::antiJam){
            counter+=Misc::DELAY;
            if(Motor::intake.get_actual_velocity() == 0 && counter > 300) stuck = true;
            if (stuck == true) {
                // TaskHandler::colorSort = false;
                // TaskHandler::autoIntake = false;
                Motor::intake.move(-127);
                pros::delay(100);
                Motor::intake.move(127);
                stuck = false;
                counter = 0;  
                // TaskHandler::colorSort = true;
                // TaskHandler::autoIntake = true;
            }
            // if(std::abs(Motor::intake.get_target_velocity()) - std::abs(Motor::intake.get_actual_velocity()) > 450 && counter > 200){
            //     Motor::intake.move(-80);
            //     pros::delay(250);
            //     Motor::intake.move(127);
            //     counter = 0;
            // }
        }
    }
}

//<------------------------------------------------------------- Color Sort ------------------------------------------------------------->
namespace Color {
    enum class colorVals { NONE, BLUE, RED };
    colorVals state = colorVals::NONE;
    bool isDone = false;
    constexpr double rLow = 8.0, rHigh = 20.0, bLow = 155.0, bHigh = 230.0, minProx= 80; // Values for colorSort
    constexpr double rLow1 = 8.0, rHigh1 = 22.0, bLow1 = 140.0, bHigh1 = 230.0, minProx1 = 70; // Values for ring store on intake
    inline bool isRed(double h, double low, double max) { return h > low && h < max; }
    inline bool isBlue(double h, double low, double max) { return h > low && h < max; }
    inline bool withinProx(int input, double max) { return (input > max); }
    colorVals colorConvertor(colorVals input) { return (input == colorVals::BLUE) ? colorVals::RED : colorVals::BLUE; }
    void colorSort(colorVals input) {
        colorVals lastColor = colorVals::NONE;
        // Sensor::o_colorSort.set_led_pwm(100);
        // Sensor::o_colorSort.set_integration_time(10);
        // controller.clear();
        if(TaskHandler::colorSort){
            // if(isRed(Sensor::o_colorSort.get_hue(),rLow,rHigh) && withinProx(Sensor::o_colorSort.get_proximity(),minProx)) { lastColor = colorVals::RED; } 
            // else if(isBlue(Sensor::o_colorSort.get_hue(),bLow,bHigh) && withinProx(Sensor::o_colorSort.get_proximity(),minProx)) { lastColor = colorVals::BLUE; }
            if(isRed(Sensor::o_colorSort.get_hue(),rLow,rHigh)) { lastColor = colorVals::RED; } 
            else if(isBlue(Sensor::o_colorSort.get_hue(),bLow,bHigh)) { lastColor = colorVals::BLUE; }
            if(input == lastColor){
                TaskHandler::dIntake = false;
                pros::delay(10);
                Motor::intake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
                Motor::intake.brake();
                // Motor::intake.move(-5);
                pros::delay(205);
                TaskHandler::dIntake = true;
                Motor::intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
                // Motor::intake.move(127);
                Motor::intake.move(TaskHandler::sharedSpeed);
                lastColor = colorVals::NONE;
            } 
        }
    }

    void toPos(Color::colorVals input){ // Color to be hovered
        if(TaskHandler::autoIntake){
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
    }
} // namespace Color

// <------------------------------------------------------------- Tier Three ------------------------------------------------------------->
namespace Hang{
    constexpr int UNWRAP_TIME = 750, TARGET_RELEASE = 4180, DIST_SENSED = 15, TARGET_PULL = -4350, TARGET_PULL2 = -4350;
    double currPos = 0, kP = 0.37;

    void pull(){
        int timer = 0;
        leftMotors.set_zero_position_all(0.0);
        rightMotors.set_zero_position_all(0.0);
        TaskHandler::isDriver = false;
        leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        do{
            leftMotors.move(-127);
            rightMotors.move(-127);
            if(timer > 2000) break;
            timer+=Misc::DELAY;
            pros::delay(Misc::DELAY);
        }
        while(leftMotors.get_position() > TARGET_PULL);
        leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
        rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
        leftMotors.brake();
        rightMotors.brake();
    }
    
    void release(){
        int timer = 0;
        // controller.clear();
        TaskHandler::lbD = true;
        TaskHandler::isDriver = false;
        leftMotors.set_zero_position_all(0.0);
        rightMotors.set_zero_position_all(0.0);
        leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        // leftMotors.move(75);
        // rightMotors.move(75);
        // pros::delay(100);
        Lift::setState(850);
        do{
            if(timer > 1600) break;
            currPos = leftMotors.get_position();
            if (std::abs(TARGET_RELEASE - currPos) < 5) break;
            double velocity = (TARGET_RELEASE - currPos)*kP;
            //leftMotors.move(127);
            //rightMotors.move(127);
            leftMotors.move(velocity);
            rightMotors.move(velocity);
            timer+=Misc::DELAY;
            pros::delay(Misc::DELAY);
        }
        while(currPos < TARGET_RELEASE);
        // controller.set_text(0, 0, "Pos: " + std::to_string(leftMotors.get_position()));
        leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        // Motor::lbL.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        // Motor::lbR.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        // Lift::setState(400);
        // pros::delay(200);
        TaskHandler::lbD = false;
        Motor::lbL.move(-127);
        Motor::lbR.move(-127);
        pros::delay(175);
        Motor::lbL.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        Motor::lbR.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        Motor::lbL.brake();
        Motor::lbR.brake();
        // Motor::lbL.move(-25);
        // Motor::lbR.move(-25);
        TaskHandler::isDriver = true;
        if(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) < -50) return;
    }

    void first(){
        int timer = 0;
        leftMotors.set_zero_position_all(0.0);
        rightMotors.set_zero_position_all(0.0);
        TaskHandler::isDriver = false;
        leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        do{
            leftMotors.move(-127);
            rightMotors.move(-127);
            if(timer > 4000) break;
            timer+=Misc::DELAY;
            pros::delay(Misc::DELAY);
        }
        while(leftMotors.get_position() > TARGET_PULL2);
        leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
        rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
        leftMotors.brake();
        rightMotors.brake();
    }

    void release2(){
        int timer = 0;
        // controller.clear();
        TaskHandler::lbD = true;
        TaskHandler::isDriver = false;
        leftMotors.set_zero_position_all(0.0);
        rightMotors.set_zero_position_all(0.0);
        leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        // leftMotors.move(75);
        // rightMotors.move(75);
        // pros::delay(100);
        Lift::setState(850);
        do{
            if(timer > 500) break;
            currPos = leftMotors.get_position();
            if (std::abs(TARGET_RELEASE - currPos) < 5) break;
            double velocity = (TARGET_RELEASE - currPos)*kP;
            //leftMotors.move(127);
            //rightMotors.move(127);
            leftMotors.move(velocity);
            rightMotors.move(velocity);
            timer+=Misc::DELAY;
            pros::delay(Misc::DELAY);
        }
        while(currPos < TARGET_RELEASE);
        // controller.set_text(0, 0, "Pos: " + std::to_string(leftMotors.get_position()));
        leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        // Motor::lbL.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        // Motor::lbR.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        // Lift::setState(400);
        // pros::delay(200);
        TaskHandler::lbD = false;
        Motor::lbL.move(-127);
        Motor::lbR.move(-127);
        pros::delay(175);
        Motor::lbL.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        Motor::lbR.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        Motor::lbL.brake();
        Motor::lbR.brake();
        // Motor::lbL.move(-25);
        // Motor::lbR.move(-25);
        TaskHandler::isDriver = true;
        if(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) < -50) return;
    }
} // namespace Hang

// <-------------------------------------------------------------- Auto Routes ----------------------------------------------------------->
namespace Auton{
    extern int state = 0;
    namespace Test{
        void main() { 

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
            void negNormal(){
                Color::state = Color::colorVals::BLUE;
                chassis.setPose(-52.16,11.95,-110.9);
                Misc::cdrift(55,55);
                Lift::setState(1500);
                Piston::lightsaberL.set_value(true);
                Motor::intake.move(-70);
                pros::delay(200);
                Misc::cdrift(0,0);
                Piston::lightsaberL.set_value(false);
                pros::delay(300);
                Motor::intake.move(0);
                Misc::cdrift(0,0);
                Lift::setState(0);
                pros::delay(50);
                chassis.moveToPoint(-21, 24, 1070, {.forwards = false,.maxSpeed=127,.minSpeed = 20,.earlyExitRange=1});
                chassis.waitUntilDone();
                Piston::mogo.set_value(true);
                pros::delay(100);
            
                Motor::intake.move(127);
                chassis.turnToPoint(-7, 38, 650, {.forwards = true,.maxSpeed=127,.minSpeed=20,.earlyExitRange=1});
                chassis.moveToPoint(-7, 38, 800, {.forwards = true,.maxSpeed=127,.minSpeed = 47.5,.earlyExitRange=1});

                chassis.turnToPoint(-5.5, 46, 200, {.forwards = true,.maxSpeed=127,.minSpeed=25,.earlyExitRange=2});
                chassis.moveToPoint(-5.5, 46, 600, {.forwards = true,.maxSpeed=127,.minSpeed = 25,.earlyExitRange=2});
                chassis.waitUntilDone();

                Misc::cdrift(25,127,200);
                chassis.turnToPoint(-25, 45, 450, {.forwards = true,.maxSpeed=127,.minSpeed=20,.earlyExitRange=2});
                chassis.moveToPoint(-25, 45, 1600, {.forwards = true,.maxSpeed=127,.minSpeed = 20,.earlyExitRange=2});

                chassis.turnToPoint(-50, 54, 200, {.forwards = true,.maxSpeed=127,.minSpeed=20,.earlyExitRange=2});
                chassis.moveToPoint(-50, 54, 900, {.forwards = true,.maxSpeed=127,.minSpeed = 20,.earlyExitRange=2});
                chassis.waitUntilDone();
                TaskHandler::antiJam = false;
                chassis.turnToHeading(315,900);
                chassis.waitUntilDone();
                Misc::cdrift(69,69,1000);
                Misc::cdrift(-25,-25,300);
                Misc::cdrift(60,60,550);
                Misc::cdrift(-25,-25,200);
                chassis.turnToHeading(135,1000);
                chassis.waitUntilDone();
                Motor::intake.move(0);
                chassis.moveToPoint(-28, 28, 1500, {.forwards = true,.maxSpeed=127,.minSpeed = 20,.earlyExitRange=2});
                chassis.turnToHeading(135,1000);
                Lift::setState(1100);
                chassis.waitUntilDone();
                Misc::cdrift(30,30);
                pros::delay(700);
                Motor::lbL.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
                Motor::lbR.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
                pros::delay(1400);
                Misc::cdrift(0,0);
            }

            void negNormal2(){
                Color::state = Color::colorVals::BLUE;
                chassis.setPose(-52.16,11.95,-110.9);
                Misc::cdrift(55,55);
                Lift::setState(1500);
                Piston::lightsaberL.set_value(true);
                Motor::intake.move(-70);
                pros::delay(200);
                Misc::cdrift(0,0);
                Piston::lightsaberL.set_value(false);
                pros::delay(300);
                Motor::intake.move(0);
                Misc::cdrift(0,0);
                Lift::setState(0);
                pros::delay(50);
                chassis.moveToPoint(-20, 25, 1300, {.forwards = false,.maxSpeed=90,.minSpeed = 20,.earlyExitRange=1});
                chassis.waitUntilDone();
                Piston::mogo.set_value(true);
                pros::delay(100);
                Motor::intake.move(127);
                chassis.turnToPoint(-7, 38, 650, {.forwards = true,.maxSpeed=127,.minSpeed=20,.earlyExitRange=1});
                chassis.moveToPoint(-7, 38, 800, {.forwards = true,.maxSpeed=127,.minSpeed=25,.earlyExitRange=1});

                chassis.turnToPoint(-5.5, 56, 200, {.forwards = true,.maxSpeed=127,.minSpeed=25,.earlyExitRange=2});
                chassis.moveToPoint(-5.5, 56, 1100, {.forwards = true,.maxSpeed=90,.minSpeed=25,.earlyExitRange=2});
                chassis.waitUntilDone();
                pros::delay(275);

                chassis.moveToPoint(-17, 33, 600, {.forwards = false,.maxSpeed=127,.minSpeed = 25,.earlyExitRange=2});
                chassis.turnToPoint(-24, 49, 450, {.forwards = true,.maxSpeed=127,.minSpeed=20,.earlyExitRange=2});
                chassis.moveToPoint(-24, 49, 1600, {.forwards = true,.maxSpeed=127,.minSpeed=20,.earlyExitRange=2});

                chassis.turnToPoint(-48, 50, 900, {.forwards = true,.maxSpeed=127,.minSpeed=20,.earlyExitRange=2});
                chassis.moveToPoint(-48, 50, 1200, {.forwards = true,.maxSpeed=90,.minSpeed=20,.earlyExitRange=2});
                chassis.waitUntilDone();
                TaskHandler::antiJam = false;
                // chassis.turnToHeading(315,900);
                chassis.turnToPoint(-74, 74, 1000, {.forwards = true,.maxSpeed=127,.minSpeed=20,.earlyExitRange=2});
                chassis.waitUntilDone();
                Misc::cdrift(69,69,850);
                Misc::cdrift(-25,-25,450);
                // Misc::cdrift(60,60,550);
                // Misc::cdrift(-25,-25,200);
                chassis.turnToHeading(135,1000);
                chassis.waitUntilDone();
                Motor::intake.move(0);
                chassis.moveToPoint(-28, 28, 1500, {.forwards = true,.maxSpeed=127,.minSpeed = 20,.earlyExitRange=2});
                chassis.turnToHeading(135,1000);
                Lift::setState(1100);
                chassis.waitUntilDone();
                Misc::cdrift(30,30);
                pros::delay(700);
                Motor::lbL.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
                Motor::lbR.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
                pros::delay(1400);
                Misc::cdrift(0,0);
            }

            void pos(){
                Color::state = Color::colorVals::BLUE;
                chassis.setPose(-51.53,-10.35,-68.4);
                Misc::cdrift(55,55);
                Lift::setState(1500);
                Piston::lightsaberL.set_value(true);
                Motor::intake.move(-70);
                pros::delay(200);
                Misc::cdrift(0,0);
                Piston::lightsaberL.set_value(false);
                pros::delay(300);
                Motor::intake.move(0);
                Misc::cdrift(0,0);
                Lift::setState(0);
                pros::delay(50);

                chassis.moveToPoint(-20, -25, 1200, {.forwards = false,.maxSpeed=100,.minSpeed = 20,.earlyExitRange=1});
                chassis.waitUntilDone();
                Piston::mogo.set_value(true);
                pros::delay(100);
                Motor::intake.move(127);
                TaskHandler::antiJam = false;

                chassis.turnToPoint(-24, -52, 650, {.forwards = true,.maxSpeed=127,.minSpeed=20,.earlyExitRange=2});
                chassis.moveToPoint(-24, -52, 800, {.forwards = true,.maxSpeed=127,.minSpeed = 20,.earlyExitRange=2}); 
                chassis.waitUntilDone();
                pros::delay(250);
                // chassis.turnToHeading(340,800,{.earlyExitRange=2});
                // chassis.waitUntilDone();
                chassis.moveToPoint(-48, -24, 800, {.forwards = false,.maxSpeed=127,.minSpeed = 20,.earlyExitRange=2}); 
                chassis.waitUntilDone();
                Piston::mogo.set_value(false);
                pros::delay(50);
                TaskHandler::antiJam = true;

                chassis.turnToPoint(-6.5, -45.5, 900, {.forwards = false,.maxSpeed=127,.minSpeed=20,.earlyExitRange=2});
                chassis.moveToPoint(-6.5, -45.5, 1750, {.forwards = false,.maxSpeed=65,.minSpeed = 20,.earlyExitRange=2}); 
                chassis.waitUntilDone();
                Piston::mogo.set_value(true);
                pros::delay(100);
                Motor::intake.move(127);


                // chassis.turnToPoint(-48, -50, 900, {.forwards = true,.maxSpeed=127,.minSpeed=20,.earlyExitRange=2});
                // chassis.moveToPoint(-48, -50, 1500, {.forwards = true,.maxSpeed=127,.minSpeed=20,.earlyExitRange=2});
                // chassis.waitUntilDone();
                // chassis.turnToHeading(315,900);
                chassis.turnToPoint(-74, -74, 450, {.forwards = true,.maxSpeed=127,.minSpeed=20,.earlyExitRange=2});
                chassis.moveToPoint(-74, -74, 2000, {.forwards = true,.maxSpeed=127,.minSpeed=20,.earlyExitRange=2});
                chassis.waitUntil(30);
                TaskHandler::antiJam = false;
                chassis.waitUntilDone();
                Misc::cdrift(69,69,400);
                Misc::cdrift(-25,-25,350);
                chassis.turnToHeading(135,1000,{.maxSpeed=127,.minSpeed=10,.earlyExitRange=2});
                chassis.waitUntilDone();
                // Motor::intake.move(0);
                chassis.moveToPoint(-28, -28, 1500, {.forwards = true,.maxSpeed=127,.minSpeed = 20,.earlyExitRange=2});
                chassis.turnToHeading(45,1000,{.maxSpeed=127,.minSpeed=10,.earlyExitRange=2});
                chassis.waitUntilDone();
                Lift::setState(1100);
                // chassis.waitUntilDone();
                Misc::cdrift(30,30);
                pros::delay(700);
                Motor::lbL.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
                Motor::lbR.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
                pros::delay(1400);
                Misc::cdrift(0,0);

            }
            void solo(){
                Color::state = Color::colorVals::BLUE;
                chassis.setPose(-52.16,11.95,-110.9);
                Misc::cdrift(55,55);
                Lift::setState(1500);
                Piston::lightsaberL.set_value(true);
                Motor::intake.move(-70);
                pros::delay(200);
                Misc::cdrift(0,0);
                Piston::lightsaberL.set_value(false);
                pros::delay(300);
                Motor::intake.move(0);
                Misc::cdrift(0,0);
                Lift::setState(0);
                pros::delay(50);
                chassis.moveToPoint(-21, 24, 1070, {.forwards = false,.maxSpeed=127,.minSpeed = 20,.earlyExitRange=1});
                chassis.waitUntilDone();
                Piston::mogo.set_value(true);
                pros::delay(100);
                Motor::intake.move(127);
                chassis.turnToPoint(-7, 38, 650, {.forwards = true,.maxSpeed=127,.minSpeed=20,.earlyExitRange=1});
                chassis.moveToPoint(-7, 38, 800, {.forwards = true,.maxSpeed=127,.minSpeed = 47.5,.earlyExitRange=1});

                chassis.turnToPoint(-5.5, 46, 200, {.forwards = true,.maxSpeed=127,.minSpeed=25,.earlyExitRange=2});
                chassis.moveToPoint(-5.5, 46, 600, {.forwards = true,.maxSpeed=127,.minSpeed = 25,.earlyExitRange=2});
                chassis.waitUntilDone();

                Misc::cdrift(25,127,200);
                chassis.turnToPoint(-25, 45, 450, {.forwards = true,.maxSpeed=127,.minSpeed=20,.earlyExitRange=2});
                chassis.moveToPoint(-25, 45, 1600, {.forwards = true,.maxSpeed=127,.minSpeed = 20,.earlyExitRange=2});

                // chassis.turnToPoint(-43, 11, 350, {.forwards = true,.maxSpeed=127,.minSpeed=20,.earlyExitRange=2});
                // chassis.moveToPoint(-43, 11, 1200, {.forwards = true,.maxSpeed=127,.minSpeed = 10,.earlyExitRange=2});
                chassis.moveToPoint(-52, -17, 2100, {.forwards = true,.maxSpeed=127,.minSpeed = 10,.earlyExitRange=2});
                chassis.waitUntil(29);
                Piston::mogo.set_value(false);

                TaskHandler::colorSort = false;
                TaskHandler::autoIntake = true;
                Motor::intake.move(90);
                chassis.cancelMotion();
                chassis.moveToPoint(-54, -17, 2100, {.forwards = true,.maxSpeed=69,.minSpeed = 10,.earlyExitRange=2});
                
                // chassis.turnToPoint(-52, 14, 200, {.forwards = true,.maxSpeed=127,.minSpeed=20,.earlyExitRange=2});
                
                chassis.waitUntilDone();
                pros::delay(200);

                chassis.turnToPoint(-20, -26, 900, {.forwards = false,.maxSpeed=127,.minSpeed=0});
                chassis.waitUntilDone();
                // Motor::intake.move(0);
                chassis.moveToPoint(-20, -26, 1200, {.forwards = false,.maxSpeed=127,.minSpeed = 10,.earlyExitRange=1});
                chassis.waitUntilDone();
                Piston::mogo.set_value(true);
                pros::delay(100);
                Motor::intake.move(127);
                TaskHandler::colorSort = true;
                TaskHandler::autoIntake = false;


                chassis.turnToPoint(-25, -52, 900, {.forwards = true,.maxSpeed=127,.minSpeed=0});
                chassis.waitUntilDone();
                Lift::setState(1100);
                chassis.moveToPoint(-25, -52, 1200, {.forwards = true,.maxSpeed=127,.minSpeed = 10,.earlyExitRange=1});
                chassis.waitUntilDone();
                pros::delay(400);
                chassis.turnToPoint(-18, -22, 900, {.forwards = true,.maxSpeed=127,.minSpeed=0});
                chassis.moveToPoint(-18, -22, 1200, {.forwards = true,.maxSpeed=127,.minSpeed = 10,.earlyExitRange=1});
                // chassis.turnToPoint(-15, -18, 350, {.forwards = true,.maxSpeed=127,.minSpeed=0});
                // chassis.moveToPoint(-15, -18, 1200, {.forwards = true,.maxSpeed=127,.minSpeed = 10,.earlyExitRange=1});
                Misc::cdrift(30,25,1500);
                // Lift::setState(1100);
            }
            void solo2(){
                Color::state = Color::colorVals::BLUE;
                chassis.setPose(-52.16,11.95,-110.9);
                Misc::cdrift(46,46);
                Lift::setState(1500);
                Piston::lightsaberL.set_value(true);
                Motor::intake.move(-70);
                pros::delay(200);
                Misc::cdrift(0,0);
                Piston::lightsaberL.set_value(false);
                pros::delay(300);
                Motor::intake.move(0);
                Lift::setState(0);
                chassis.moveToPoint(-22, 25, 2000,{.forwards = false,.maxSpeed = 75});
                chassis.waitUntilDone();
                Piston::mogo.set_value(true);
                pros::delay(100);
                Motor::intake.move(127);

                chassis.turnToPoint(-7, 40, 650, {.forwards = true,.maxSpeed=127,.minSpeed=0});
                chassis.moveToPoint(-7, 40, 1500, {.forwards = true,.maxSpeed=127,.minSpeed = 10,.earlyExitRange=1});

                // chassis.turnToPoint(-12, 29, 650, {.forwards = true,.maxSpeed=127,.minSpeed=0});
                // chassis.moveToPoint(-12, 29, 1500, {.forwards = true,.maxSpeed=127,.minSpeed = 10,.earlyExitRange=1});

                // chassis.moveToPose(-5.5,62,0,2000,{.horizontalDrift=12,.lead=0.7,.maxSpeed=127,.minSpeed=10});
                chassis.waitUntilDone();
                pros::delay(150);

                chassis.moveToPoint(-24, 32, 1500, {.forwards = false,.maxSpeed=127,.minSpeed = 10,.earlyExitRange=1});
                chassis.turnToPoint(-23, 54, 350, {.forwards = true,.maxSpeed=127,.minSpeed=0});
                chassis.moveToPoint(-23, 54, 1200, {.forwards = true,.maxSpeed=127,.minSpeed = 10,.earlyExitRange=1});
                chassis.turnToPoint(-48, 24, 350, {.forwards = true,.maxSpeed=127,.minSpeed=0});
                chassis.moveToPoint(-48, 24, 1200, {.forwards = true,.maxSpeed=127,.minSpeed = 10,.earlyExitRange=1});

                chassis.turnToPoint(-48, -24, 800, {.forwards = true,.maxSpeed=127,.minSpeed=0});
                chassis.waitUntilDone();
                Piston::mogo.set_value(false);
                TaskHandler::colorSort = false;
                TaskHandler::autoIntake = true;
                Motor::intake.move(65);
                chassis.moveToPoint(-48, -24, 2500, {.forwards = true,.maxSpeed=70,.minSpeed = 10,.earlyExitRange=1});
                chassis.turnToPoint(-23, -22, 900, {.forwards = false,.maxSpeed=127,.minSpeed=0});
                chassis.moveToPoint(-23, -22, 1200, {.forwards = false,.maxSpeed=127,.minSpeed = 10,.earlyExitRange=1});
                chassis.waitUntilDone();
                Piston::mogo.set_value(true);
                pros::delay(100);
                Motor::intake.move(127);
                TaskHandler::colorSort = true;

                chassis.turnToPoint(-25, -50, 900, {.forwards = true,.maxSpeed=127,.minSpeed=0});
                chassis.moveToPoint(-25, -50, 1200, {.forwards = true,.maxSpeed=127,.minSpeed = 10,.earlyExitRange=1});
                chassis.waitUntilDone();
                pros::delay(150);
                chassis.turnToPoint(-15, -18, 350, {.forwards = true,.maxSpeed=127,.minSpeed=0});
                chassis.moveToPoint(-15, -18, 1200, {.forwards = true,.maxSpeed=127,.minSpeed = 10,.earlyExitRange=1});
                Lift::setState(1200);
            }
            void solo3(){
                Color::state = Color::colorVals::BLUE;
                chassis.setPose(-52.16,11.95,-110.9);
                Misc::cdrift(55,55);
                Lift::setState(1500);
                Piston::lightsaberL.set_value(true);
                Motor::intake.move(-70);
                pros::delay(200);
                Misc::cdrift(0,0);
                Piston::lightsaberL.set_value(false);
                pros::delay(300);
                Motor::intake.move(0);
                Misc::cdrift(0,0);
                Lift::setState(0);
                pros::delay(50);
                chassis.moveToPoint(-20, 25, 1200, {.forwards = false,.maxSpeed=100,.minSpeed = 20,.earlyExitRange=1});
                chassis.waitUntilDone();
                Piston::mogo.set_value(true);
                pros::delay(100);
                Motor::intake.move(127);
                chassis.turnToPoint(-7, 38, 650, {.forwards = true,.maxSpeed=127,.minSpeed=20,.earlyExitRange=1});
                chassis.moveToPoint(-7, 38, 800, {.forwards = true,.maxSpeed=127,.minSpeed=25,.earlyExitRange=1});

                chassis.turnToPoint(-5.5, 56, 200, {.forwards = true,.maxSpeed=127,.minSpeed=25,.earlyExitRange=2});
                chassis.moveToPoint(-5.5, 56, 1100, {.forwards = true,.maxSpeed=90,.minSpeed=25,.earlyExitRange=2});
                chassis.waitUntilDone();
                pros::delay(275);

                chassis.moveToPoint(-17, 33, 600, {.forwards = false,.maxSpeed=127,.minSpeed = 25,.earlyExitRange=2});
                chassis.turnToPoint(-24, 49, 450, {.forwards = true,.maxSpeed=127,.minSpeed=20,.earlyExitRange=2});
                chassis.moveToPoint(-24, 49, 1600, {.forwards = true,.maxSpeed=127,.minSpeed=20,.earlyExitRange=2});

                // chassis.turnToPoint(-43, 11, 350, {.forwards = true,.maxSpeed=127,.minSpeed=20,.earlyExitRange=2});
                // chassis.moveToPoint(-43, 11, 1200, {.forwards = true,.maxSpeed=127,.minSpeed = 10,.earlyExitRange=2});
                chassis.turnToPoint(-53, -18, 200, {.forwards = true,.maxSpeed=127,.minSpeed=20,.earlyExitRange=2});
                chassis.moveToPoint(-53, -18, 2100, {.forwards = true,.maxSpeed=127,.minSpeed = 10,.earlyExitRange=2});
                chassis.waitUntil(32);
                Piston::mogo.set_value(false);

                TaskHandler::colorSort = false;
                TaskHandler::autoIntake = true;
                TaskHandler::antiJam = false;
                Motor::intake.move(83);
                chassis.cancelMotion();
                chassis.moveToPoint(-53, -18, 2100, {.forwards = true,.maxSpeed=69,.minSpeed = 10,.earlyExitRange=2});
                
                // chassis.turnToPoint(-52, 14, 200, {.forwards = true,.maxSpeed=127,.minSpeed=20,.earlyExitRange=2});
                
                chassis.waitUntilDone();
                pros::delay(200);

                chassis.turnToPoint(-20, -23, 900, {.forwards = false,.maxSpeed=127,.minSpeed=0});
                chassis.waitUntilDone();
                // Motor::intake.move(0);
                chassis.moveToPoint(-20, -23, 1200, {.forwards = false,.maxSpeed=127,.minSpeed = 10,.earlyExitRange=1});
                chassis.waitUntilDone();
                Piston::mogo.set_value(true);
                TaskHandler::colorSort = true;
                TaskHandler::autoIntake = false;
                pros::delay(100);
                Motor::intake.move(127);


                chassis.turnToPoint(-25, -52, 900, {.forwards = true,.maxSpeed=127,.minSpeed=0});
                chassis.waitUntilDone();
                chassis.moveToPoint(-25, -52, 1200, {.forwards = true,.maxSpeed=127,.minSpeed = 10,.earlyExitRange=1});
                chassis.waitUntil(10);
                Lift::setState(1100);
                chassis.waitUntilDone();
                pros::delay(400);
                chassis.turnToPoint(-14, -22, 900, {.forwards = true,.maxSpeed=127,.minSpeed=0});
                chassis.moveToPoint(-14, -22, 1200, {.forwards = true,.maxSpeed=127,.minSpeed = 10,.earlyExitRange=1});
                chassis.waitUntil(8);
                Motor::intake.move(0);

                chassis.waitUntilDone();
                // Motor::intake.move(0);
                // chassis.turnToPoint(-15, -18, 350, {.forwards = true,.maxSpeed=127,.minSpeed=0});
                // chassis.moveToPoint(-15, -18, 1200, {.forwards = true,.maxSpeed=127,.minSpeed = 10,.earlyExitRange=1});
                Misc::cdrift(25,20,1100);
                // Lift::setState(1100);
            }
        } // namespace Qual
        namespace Elim{
            void negCloseWall(){
                Color::state = Color::colorVals::BLUE;
                chassis.setPose(-52.16,11.95,-110.9);
                Misc::cdrift(55,55);
                Lift::setState(1500);
                Piston::lightsaberL.set_value(true);
                Motor::intake.move(-70);
                pros::delay(200);
                Misc::cdrift(0,0);
                Piston::lightsaberL.set_value(false);
                pros::delay(300);
                Motor::intake.move(0);
                Misc::cdrift(0,0);
                Lift::setState(0);
                pros::delay(50);
                chassis.moveToPoint(-21, 24, 1070, {.forwards = false,.maxSpeed=127,.minSpeed = 20,.earlyExitRange=1});
                chassis.waitUntilDone();
                Piston::mogo.set_value(true);
                pros::delay(100);
                Motor::intake.move(127);
                chassis.turnToPoint(-7, 38, 650, {.forwards = true,.maxSpeed=127,.minSpeed=20,.earlyExitRange=1});
                chassis.moveToPoint(-7, 38, 800, {.forwards = true,.maxSpeed=127,.minSpeed = 55,.earlyExitRange=1});

                chassis.turnToPoint(-5.5, 46, 200, {.forwards = true,.maxSpeed=127,.minSpeed=25,.earlyExitRange=2});
                chassis.moveToPoint(-5.5, 46, 600, {.forwards = true,.maxSpeed=127,.minSpeed = 25,.earlyExitRange=2});
                chassis.waitUntilDone();

                Misc::cdrift(25,127,200);
                chassis.turnToPoint(-25, 45, 450, {.forwards = true,.maxSpeed=127,.minSpeed=20,.earlyExitRange=2});
                chassis.moveToPoint(-25, 45, 1600, {.forwards = true,.maxSpeed=127,.minSpeed = 20,.earlyExitRange=2});
                chassis.waitUntil(7);
                Lift::setState(65);
                TaskHandler::antiJam = false;
                pros::delay(200);
                chassis.waitUntilDone();

                chassis.turnToPoint(-4, 62, 1100, {.forwards = true,.maxSpeed=127,.minSpeed=10,.earlyExitRange=0});
                chassis.moveToPoint(-4, 62, 1100, {.forwards = true,.maxSpeed=127,.minSpeed=20,.earlyExitRange=2});
                chassis.waitUntilDone();
                Motor::intake.move(0);
                Lift::setState(1500);
                pros::delay(550);
                Misc::cdrift(-55,55,250);

                chassis.turnToPoint(-59, 59, 900, {.forwards = false,.maxSpeed=127,.minSpeed=20,.earlyExitRange=2});
                chassis.moveToPoint(-59, 59, 1200, {.forwards = false,.maxSpeed=127,.minSpeed = 20,.earlyExitRange=2});
                // chassis.turnToPoint(-52, 54, 200, {.forwards = false,.maxSpeed=127,.minSpeed=20,.earlyExitRange=2});
                // chassis.moveToPoint(-52, 54, 1200, {.forwards = false,.maxSpeed=127,.minSpeed = 20,.earlyExitRange=2});
                chassis.waitUntilDone();
                chassis.turnToHeading(315,900);
                chassis.waitUntilDone();
                Motor::intake.move(127);
                Misc::cdrift(69,69,800);
                Misc::cdrift(-25,-25,200);
                Misc::cdrift(60,60,400);
                Misc::cdrift(-25,-25,200);
                chassis.moveToPoint(-43, 24, 1500, {.forwards = false,.maxSpeed=127,.minSpeed = 20,.earlyExitRange=2});
                chassis.waitUntilDone();
                Lift::setState(0);
                chassis.turnToPoint(-61, -42, 650, {.forwards = true,.maxSpeed=127,.minSpeed=20,.earlyExitRange=2});
                chassis.waitUntilDone();
                TaskHandler::colorSort = true;
                TaskHandler::antiJam = false;
                Motor::intake.move(127);
                chassis.moveToPoint(-61, -42, 1500, {.forwards = true,.maxSpeed=127,.minSpeed = 20,.earlyExitRange=2});

            }

            void negCloseWall2(){
                Color::state = Color::colorVals::BLUE;
                chassis.setPose(-52.16,11.95,-110.9);
                Misc::cdrift(55,55);
                Lift::setState(1500);
                Piston::lightsaberL.set_value(true);
                Motor::intake.move(-70);
                pros::delay(200);
                Misc::cdrift(0,0);
                Piston::lightsaberL.set_value(false);
                pros::delay(300);
                Motor::intake.move(0);
                Misc::cdrift(0,0);
                Lift::setState(0);
                pros::delay(50);
                chassis.moveToPoint(-20, 25, 1200, {.forwards = false,.maxSpeed=100,.minSpeed = 20,.earlyExitRange=1});
                chassis.waitUntilDone();
                Piston::mogo.set_value(true);
                pros::delay(100);
                Motor::intake.move(127);
                chassis.turnToPoint(-7, 38, 650, {.forwards = true,.maxSpeed=127,.minSpeed=20,.earlyExitRange=1});
                chassis.moveToPoint(-7, 38, 800, {.forwards = true,.maxSpeed=127,.minSpeed=25,.earlyExitRange=1});

                chassis.turnToPoint(-5.5, 52, 200, {.forwards = true,.maxSpeed=127,.minSpeed=25,.earlyExitRange=2});
                chassis.moveToPoint(-5.5, 52, 1000, {.forwards = true,.maxSpeed=90,.minSpeed=25,.earlyExitRange=2});
                chassis.waitUntilDone();
                pros::delay(250);

                chassis.moveToPoint(-17, 33, 600, {.forwards = false,.maxSpeed=127,.minSpeed = 25,.earlyExitRange=2});
                chassis.turnToPoint(-24, 49, 450, {.forwards = true,.maxSpeed=127,.minSpeed=20,.earlyExitRange=2});
                chassis.moveToPoint(-24, 49, 1600, {.forwards = true,.maxSpeed=127,.minSpeed=20,.earlyExitRange=2});
                chassis.waitUntil(9);
                Lift::setState(66);
                TaskHandler::antiJam = false;
                // pros::delay(200);
                chassis.waitUntilDone();
                pros::delay(225);

                chassis.turnToPoint(-4.5, 61, 1100, {.forwards = true,.maxSpeed=127,.minSpeed=10,.earlyExitRange=0});
                chassis.waitUntilDone();
                // chassis.waitUntilDone();
                chassis.moveToPoint(-4.5, 61, 1200, {.forwards = true,.maxSpeed=90,.minSpeed=20,.earlyExitRange=0});
                chassis.waitUntilDone();
                Motor::intake.move(0);
                Lift::setState(1500);
                pros::delay(550);
                Misc::cdrift(-75,-75,450);

                chassis.turnToPoint(-48, 50, 900, {.forwards = true,.maxSpeed=127,.minSpeed=20,.earlyExitRange=2});
                chassis.moveToPoint(-48, 50, 1200, {.forwards = true,.maxSpeed=90,.minSpeed=20,.earlyExitRange=2});
                chassis.waitUntilDone();
                TaskHandler::antiJam = false;
                // chassis.turnToHeading(315,900);
                chassis.turnToPoint(-74, 74, 1000, {.forwards = true,.maxSpeed=127,.minSpeed=20,.earlyExitRange=2});
                chassis.waitUntilDone();
                Misc::cdrift(69,69,1050);
                Misc::cdrift(-25,-25,300);
                Misc::cdrift(60,60,550);
                Misc::cdrift(-25,-25,200);
                chassis.moveToPoint(-43, 24, 1500, {.forwards = false,.maxSpeed=127,.minSpeed = 20,.earlyExitRange=2});
                chassis.waitUntilDone();
                Lift::setState(0);
                chassis.turnToPoint(-61, -42, 650, {.forwards = true,.maxSpeed=127,.minSpeed=20,.earlyExitRange=2});
                chassis.waitUntilDone();
                TaskHandler::colorSort = true;
                TaskHandler::antiJam = false;
                Motor::intake.move(127);
                chassis.moveToPoint(-61, -42, 1500, {.forwards = true,.maxSpeed=127,.minSpeed = 20,.earlyExitRange=2});
                
            }
            void pos(){//3 Top Ring (2 Mogo & 1 Wall stake)
                Color::state = Color::colorVals::BLUE;
                TaskHandler::autoIntake = true;
                chassis.setPose(-57,-36, 108);
                Motor::lbL.set_zero_position(165);
                Motor::lbL.set_zero_position(165);
                
                Lift::setState(900);
                Piston::lightsaberL.set_value(true);
                Motor::intake.move(65);
                TaskHandler::autoIntake = true;
                Misc::linear(42, 2500, {.forwards = true,.maxSpeed=127,.minSpeed = 10,.earlyExitRange=1});
                chassis.waitUntilDone();
                Piston::saberclamp.set_value(true);
                pros::delay(150);
                Misc::cdrift(-45,-45,450);
                Piston::saberclamp.set_value(false);
                chassis.turnToPoint(-10, -42, 900, {.forwards = false,.maxSpeed=127,.minSpeed=20,.earlyExitRange=2});
                chassis.moveToPoint(-10, -42, 1200, {.forwards = false,.maxSpeed=90,.minSpeed=20,.earlyExitRange=2});
                chassis.waitUntilDone();
                Piston::mogo.set_value(true);
                Piston::lightsaberL.set_value(false);
                pros::delay(100);
                TaskHandler::autoIntake = false;
                Motor::intake.move(127);
                Misc::cdrift(10,75,450);

                chassis.turnToPoint(-4.5, -61, 900, {.forwards = true,.maxSpeed=127,.minSpeed=20,.earlyExitRange=2});
                chassis.moveToPoint(-4.5, -61, 1200, {.forwards = true,.maxSpeed=90,.minSpeed=20,.earlyExitRange=2});
                chassis.waitUntilDone();
                Motor::intake.move(0);
                Lift::setState(1500);
                pros::delay(550);
                Misc::cdrift(-75,-75,450);

                chassis.turnToPoint(-48, -50, 900, {.forwards = true,.maxSpeed=127,.minSpeed=20,.earlyExitRange=2});
                chassis.moveToPoint(-48, -50, 1200, {.forwards = true,.maxSpeed=90,.minSpeed=20,.earlyExitRange=2});
                chassis.waitUntilDone();
                // chassis.turnToHeading(315,900);
                chassis.turnToPoint(-74, -74, 450, {.forwards = true,.maxSpeed=127,.minSpeed=20,.earlyExitRange=2});
                chassis.moveToPoint(-74, -74, 2000, {.forwards = true,.maxSpeed=127,.minSpeed=20,.earlyExitRange=2});
                chassis.waitUntil(30);
                TaskHandler::antiJam = false;
                TaskHandler::autoIntake = true;
                chassis.waitUntilDone();
                Misc::cdrift(69,69,400);
                Misc::cdrift(-25,-25,350);
                // chassis.turnToHeading(135,1000,{.maxSpeed=127,.minSpeed=10,.earlyExitRange=2});
                chassis.waitUntilDone();
                // Motor::intake.move(0);
                chassis.moveToPoint(-22, -22, 1500, {.forwards = false,.maxSpeed=127,.minSpeed = 20,.earlyExitRange=2});
                chassis.waitUntilDone();
                Piston::mogo.set_value(true);
                pros::delay(100);
                TaskHandler::autoIntake = false;
                Motor::intake.move(127);

            }
        } // namespace Elim
    } // namespace Red
    namespace Blue{
        namespace Qual{
            void negNormal2(){
                Color::state = Color::colorVals::BLUE;
                chassis.setPose(52.16,11.95,110.9);
                Misc::cdrift(55,55);
                Lift::setState(1500);
                Piston::lightsaberL.set_value(true);
                Motor::intake.move(-70);
                pros::delay(200);
                Misc::cdrift(0,0);
                Piston::lightsaberL.set_value(false);
                pros::delay(300);
                Motor::intake.move(0);
                Misc::cdrift(0,0);
                Lift::setState(0);
                pros::delay(50);
                chassis.moveToPoint(20, 25, 1300, {.forwards = false,.maxSpeed=90,.minSpeed = 20,.earlyExitRange=1});
                chassis.waitUntilDone();
                Piston::mogo.set_value(true);
                pros::delay(100);
                Motor::intake.move(127);
                chassis.turnToPoint(7, 38, 650, {.forwards = true,.maxSpeed=127,.minSpeed=20,.earlyExitRange=1});
                chassis.moveToPoint(7, 38, 800, {.forwards = true,.maxSpeed=127,.minSpeed=25,.earlyExitRange=1});

                chassis.turnToPoint(5.5, 56, 200, {.forwards = true,.maxSpeed=127,.minSpeed=25,.earlyExitRange=2});
                chassis.moveToPoint(5.5, 56, 1100, {.forwards = true,.maxSpeed=90,.minSpeed=25,.earlyExitRange=2});
                chassis.waitUntilDone();
                pros::delay(275);

                chassis.moveToPoint(17, 33, 600, {.forwards = false,.maxSpeed=127,.minSpeed = 25,.earlyExitRange=2});
                chassis.turnToPoint(24, 49, 450, {.forwards = true,.maxSpeed=127,.minSpeed=20,.earlyExitRange=2});
                chassis.moveToPoint(24, 49, 1600, {.forwards = true,.maxSpeed=127,.minSpeed=20,.earlyExitRange=2});

                chassis.turnToPoint(48, 50, 900, {.forwards = true,.maxSpeed=127,.minSpeed=20,.earlyExitRange=2});
                chassis.moveToPoint(48, 50, 1200, {.forwards = true,.maxSpeed=90,.minSpeed=20,.earlyExitRange=2});
                chassis.waitUntilDone();
                TaskHandler::antiJam = false;
                // chassis.turnToHeading(315,900);
                chassis.turnToPoint(74, 74, 1000, {.forwards = true,.maxSpeed=127,.minSpeed=20,.earlyExitRange=2});
                chassis.waitUntilDone();
                Misc::cdrift(69,69,850);
                Misc::cdrift(-25,-25,450);
                // Misc::cdrift(60,60,550);
                // Misc::cdrift(-25,-25,200);
                chassis.turnToHeading(225,1000);
                chassis.waitUntilDone();
                Motor::intake.move(0);
                chassis.moveToPoint(28, 28, 1500, {.forwards = true,.maxSpeed=127,.minSpeed = 20,.earlyExitRange=2});
                chassis.turnToHeading(225,1000);
                Lift::setState(1100);
                chassis.waitUntilDone();
                Misc::cdrift(30,30);
                pros::delay(700);
                Motor::lbL.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
                Motor::lbR.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
                pros::delay(1400);
                Misc::cdrift(0,0);
            }
            void pos(){

            }
            void solo3(){
                Color::state = Color::colorVals::BLUE;
                chassis.setPose(52.16,11.95,110.9);
                Misc::cdrift(55,55);
                Lift::setState(1500);
                Piston::lightsaberL.set_value(true);
                Motor::intake.move(-70);
                pros::delay(200);
                Misc::cdrift(0,0);
                Piston::lightsaberL.set_value(false);
                pros::delay(300);
                Motor::intake.move(0);
                Misc::cdrift(0,0);
                Lift::setState(0);
                pros::delay(50);
                chassis.moveToPoint(20, 25, 1200, {.forwards = false,.maxSpeed=100,.minSpeed = 20,.earlyExitRange=1});
                chassis.waitUntilDone();
                Piston::mogo.set_value(true);
                pros::delay(100);
                Motor::intake.move(127);
                chassis.turnToPoint(7, 38, 650, {.forwards = true,.maxSpeed=127,.minSpeed=20,.earlyExitRange=1});
                chassis.moveToPoint(7, 38, 800, {.forwards = true,.maxSpeed=127,.minSpeed=25,.earlyExitRange=1});

                chassis.turnToPoint(5.5, 56, 200, {.forwards = true,.maxSpeed=127,.minSpeed=25,.earlyExitRange=2});
                chassis.moveToPoint(5.5, 56, 1100, {.forwards = true,.maxSpeed=90,.minSpeed=25,.earlyExitRange=2});
                chassis.waitUntilDone();
                pros::delay(275);

                chassis.moveToPoint(17, 33, 600, {.forwards = false,.maxSpeed=127,.minSpeed = 25,.earlyExitRange=2});
                chassis.turnToPoint(24, 49, 450, {.forwards = true,.maxSpeed=127,.minSpeed=20,.earlyExitRange=2});
                chassis.moveToPoint(24, 49, 1600, {.forwards = true,.maxSpeed=127,.minSpeed=20,.earlyExitRange=2});

                // chassis.turnToPoint(-43, 11, 350, {.forwards = true,.maxSpeed=127,.minSpeed=20,.earlyExitRange=2});
                // chassis.moveToPoint(-43, 11, 1200, {.forwards = true,.maxSpeed=127,.minSpeed = 10,.earlyExitRange=2});
                chassis.turnToPoint(53, -18, 200, {.forwards = true,.maxSpeed=127,.minSpeed=20,.earlyExitRange=2});
                chassis.moveToPoint(53, -18, 2100, {.forwards = true,.maxSpeed=127,.minSpeed = 10,.earlyExitRange=2});
                chassis.waitUntil(32);
                Piston::mogo.set_value(false);

                TaskHandler::colorSort = false;
                TaskHandler::autoIntake = true;
                TaskHandler::antiJam = false;
                Motor::intake.move(83);
                chassis.cancelMotion();
                chassis.moveToPoint(53, -18, 2100, {.forwards = true,.maxSpeed=69,.minSpeed = 10,.earlyExitRange=2});
                
                // chassis.turnToPoint(-52, 14, 200, {.forwards = true,.maxSpeed=127,.minSpeed=20,.earlyExitRange=2});
                
                chassis.waitUntilDone();
                pros::delay(200);

                chassis.turnToPoint(20, -23, 900, {.forwards = false,.maxSpeed=127,.minSpeed=0});
                chassis.waitUntilDone();
                // Motor::intake.move(0);
                chassis.moveToPoint(20, -23, 1200, {.forwards = false,.maxSpeed=127,.minSpeed = 10,.earlyExitRange=1});
                chassis.waitUntilDone();
                Piston::mogo.set_value(true);
                TaskHandler::colorSort = true;
                TaskHandler::autoIntake = false;
                pros::delay(100);
                Motor::intake.move(127);


                chassis.turnToPoint(25, -52, 900, {.forwards = true,.maxSpeed=127,.minSpeed=0});
                chassis.waitUntilDone();
                chassis.moveToPoint(25, -52, 1200, {.forwards = true,.maxSpeed=127,.minSpeed = 10,.earlyExitRange=1});
                chassis.waitUntil(10);
                Lift::setState(1100);
                chassis.waitUntilDone();
                pros::delay(400);
                chassis.turnToPoint(14, -22, 900, {.forwards = true,.maxSpeed=127,.minSpeed=0});
                chassis.moveToPoint(14, -22, 1200, {.forwards = true,.maxSpeed=127,.minSpeed = 10,.earlyExitRange=1});
                chassis.waitUntil(8);
                Motor::intake.move(0);

                chassis.waitUntilDone();
                // Motor::intake.move(0);
                // chassis.turnToPoint(-15, -18, 350, {.forwards = true,.maxSpeed=127,.minSpeed=0});
                // chassis.moveToPoint(-15, -18, 1200, {.forwards = true,.maxSpeed=127,.minSpeed = 10,.earlyExitRange=1});
                Misc::cdrift(20,25,1100);
                // Lift::setState(1100);
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
            Color::state = Color::colorVals::BLUE;
            chassis.setPose(-57.7,-5.8,325);
            Lift::setState(1500);
            Motor::intake.move(-60);
            Piston::lightsaberL.set_value(true);
            pros::delay(200);
            Piston::lightsaberL.set_value(false);
            pros::delay(300);
            Motor::intake.move(0);
            Lift::setState(0);
            chassis.moveToPoint(-48, -24, 2000, {.forwards = false,.maxSpeed=127,.minSpeed = 10,.earlyExitRange=1});
            chassis.waitUntilDone();
            Piston::mogo.set_value(true);
            pros::delay(100);
            Motor::intake.move(127);
            chassis.turnToPoint(-23, -21, 900, {.forwards = true,.maxSpeed=127,.minSpeed=10,.earlyExitRange=1});
            chassis.moveToPoint(-23, -21, 2000, {.forwards = true,.maxSpeed=127,.minSpeed = 10,.earlyExitRange=1});
            chassis.turnToPoint(6, -55, 900, {.forwards = true,.maxSpeed=127,.minSpeed=10,.earlyExitRange=1});
            chassis.waitUntilDone();
            Misc::cdrift(75,75,400);
            Misc::cdrift(100,50);
            pros::delay(150);
            Lift::setState(65);
            pros::delay(200);
            Misc::cdrift(50,50);
            pros::delay(1800);
            Motor::intake.move(0);
            Lift::setState(1500);
            pros::delay(500);
            Lift::setState(0);
            pros::delay(150);
            chassis.moveToPoint(0, -48, 1500, {.forwards = false,.maxSpeed=127,.minSpeed = 10,.earlyExitRange=1});
            chassis.turnToPoint(-62, -48, 900, {.forwards = true,.maxSpeed=127,.minSpeed=10,.earlyExitRange=1});
            chassis.waitUntilDone();
            Motor::intake.move(127);
            chassis.moveToPoint(-62, -48, 2000, {.forwards = true,.maxSpeed=85,.minSpeed = 10,.earlyExitRange=1});
            chassis.turnToPoint(-44, -62, 900, {.forwards = true,.maxSpeed=127,.minSpeed=10,.earlyExitRange=1});
            chassis.moveToPoint(-44, -62, 2000, {.forwards = true,.maxSpeed=85,.minSpeed = 10,.earlyExitRange=1});
            pros::delay(150);
            chassis.turnToPoint(-65, -65, 900, {.forwards = false,.maxSpeed=100,.minSpeed=10,.earlyExitRange=1});
            chassis.waitUntilDone();
            Misc::cdrift(-65,-65);
            pros::delay(550);
            Piston::mogo.set_value(false);
            pros::delay(50);
            chassis.moveToPoint(-49, 0, 2000, {.forwards = true,.maxSpeed=127,.minSpeed = 10,.earlyExitRange=1});
            chassis.turnToPoint(-49, 24, 900, {.forwards = false,.maxSpeed=127,.minSpeed=10,.earlyExitRange=1});
            chassis.moveToPoint(-49, 27, 2000, {.forwards = false,.maxSpeed=75,.minSpeed = 10,.earlyExitRange=1});
            chassis.waitUntilDone();
            // pros::delay(55);
            Piston::mogo.set_value(true);
            pros::delay(100);

            // 2
            chassis.turnToPoint(-29, 23, 900, {.forwards = true,.maxSpeed=127,.minSpeed=10,.earlyExitRange=1});
            chassis.moveToPoint(-29, 23, 2000, {.forwards = true,.maxSpeed=127,.minSpeed = 10,.earlyExitRange=1});
            chassis.turnToPoint(3, 55, 900, {.forwards = true,.maxSpeed=127,.minSpeed=10,.earlyExitRange=1});
            chassis.waitUntilDone();
            Misc::cdrift(70,70,400);
            Misc::cdrift(50,100);
            pros::delay(150);
            Lift::setState(65);
            pros::delay(200);
            Misc::cdrift(50,50);
            pros::delay(1800);
            Motor::intake.move(0);
            Lift::setState(1500);
            pros::delay(500);
            Lift::setState(0);
            pros::delay(150);

            chassis.moveToPoint(0, 47, 1500, {.forwards = false,.maxSpeed=127,.minSpeed = 10,.earlyExitRange=1});
            chassis.turnToPoint(-62, 47, 900, {.forwards = true,.maxSpeed=127,.minSpeed=10,.earlyExitRange=1});
            chassis.waitUntilDone();
            Motor::intake.move(127);
            chassis.moveToPoint(-62, 47, 2000, {.forwards = true,.maxSpeed=85,.minSpeed = 10,.earlyExitRange=1});
            chassis.turnToPoint(-44, 62, 900, {.forwards = true,.maxSpeed=127,.minSpeed=10,.earlyExitRange=1});
            chassis.moveToPoint(-44, 62, 2000, {.forwards = true,.maxSpeed=85,.minSpeed = 10,.earlyExitRange=1});
            pros::delay(150);
            chassis.turnToPoint(-65, 65, 900, {.forwards = false,.maxSpeed=100,.minSpeed=10,.earlyExitRange=1});
            chassis.waitUntilDone();
            Misc::cdrift(-65,-65);
            pros::delay(550);
            Piston::mogo.set_value(false);
            pros::delay(50);

            // 3
            Motor::intake.move(60);
            chassis.moveToPoint(24, 46, 2500, {.forwards = true,.maxSpeed=85,.minSpeed = 10,.earlyExitRange=1});
            chassis.waitUntil(50);
            TaskHandler::autoIntake = true;
            chassis.waitUntilDone();
            pros::delay(150);
            chassis.turnToPoint(55, 21, 900, {.forwards = false,.maxSpeed=127,.minSpeed=10,.earlyExitRange=1});
            chassis.moveToPoint(55, 21, 2000, {.forwards = false,.maxSpeed=127,.minSpeed = 10,.earlyExitRange=1});
            chassis.waitUntilDone();
            Piston::mogo.set_value(true);
            pros::delay(100);
            TaskHandler::autoIntake = false;
            Motor::intake.move(0);
            chassis.turnToPoint(73, 65, 900, {.forwards = false,.maxSpeed=127,.minSpeed=10,.earlyExitRange=1});
            chassis.waitUntilDone();
            Piston::mogo.set_value(false);
            Misc::cdrift(-65,-65,1200);
            chassis.moveToPoint(48+Misc::X, 24, 2000, {.forwards = true,.maxSpeed=127,.minSpeed = 10,.earlyExitRange=1});
            chassis.turnToPoint(48+Misc::X, -3, 900, {.forwards = false,.maxSpeed=127,.minSpeed=10,.earlyExitRange=1});
            chassis.moveToPoint(48+Misc::X, -3, 1500, {.forwards = false,.maxSpeed=75,.minSpeed = 10,.earlyExitRange=1});
            chassis.waitUntilDone();
            Piston::mogo.set_value(true);
            pros::delay(100);
            Motor::intake.move(127);

            // 4
            chassis.turnToPoint(24+Misc::X-4, -21, 500, {.forwards = true,.maxSpeed=127,.minSpeed=10,.earlyExitRange=1});
            chassis.moveToPoint(24+Misc::X-4, -21, 2000, {.forwards = true,.maxSpeed=127,.minSpeed = 10,.earlyExitRange=1});

            chassis.turnToPoint(23+Misc::X-4, -48, 500, {.forwards = true,.maxSpeed=127,.minSpeed=10,.earlyExitRange=1});
            chassis.moveToPoint(23+Misc::X-4, -48, 2000, {.forwards = true,.maxSpeed=127,.minSpeed = 10,.earlyExitRange=1});
            
            chassis.turnToPoint(48+Misc::X-3, -44, 500, {.forwards = true,.maxSpeed=127,.minSpeed=10,.earlyExitRange=1});
            chassis.moveToPoint(48+Misc::X-3, -44, 2000, {.forwards = true,.maxSpeed=127,.minSpeed = 10,.earlyExitRange=1});

            chassis.turnToPoint(65+Misc::X, -65, 900, {.forwards = false,.maxSpeed=127,.minSpeed=10,.earlyExitRange=1});
            chassis.waitUntilDone();

            Piston::mogo.set_value(false);
            Misc::cdrift(-65,-65,550);
            Motor::intake.move(0);

            chassis.moveToPoint(0+Misc::X-3, 4, 2500, {.forwards = true,.maxSpeed=127,.minSpeed = 10,.earlyExitRange=1});
            chassis.waitUntil(75);
            Motor::intake.move(60);
            TaskHandler::autoIntake = true;
            chassis.waitUntilDone();
            chassis.turnToHeading(0,900,{.maxSpeed = 100,.minSpeed = 10,.earlyExitRange = 1});
            chassis.waitUntilDone();
            Piston::release.set_value(true);
            Lift::setState(550);
            Misc::cdrift(-55,-55,1000);
            
            Piston::pto.set_value(true);
            Misc::brakeState = pros::E_MOTOR_BRAKE_COAST;

            TaskHandler::lbD = false;
            Motor::lbL.move(-127);
            Motor::lbR.move(-127);
            pros::delay(175);
            Motor::lbL.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
            Motor::lbR.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
            Motor::lbL.brake();
            Motor::lbR.brake();
            
            Hang::pull();
            Hang::release();
            Hang::pull();
            Hang::release();
            Hang::pull();
            Hang::release2();

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
            if(TaskHandler::isDriver) {
                int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
                int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
                leftMotors.move(leftY+rightX);
                rightMotors.move(leftY-rightX);
            }
            // chassis.arcade(leftY, rightX);
            pros::delay(Misc::DELAY);
        }
    }
    void wall(){
        Lift::setState(1400);
        Motor::lbL.move(127);
        Motor::lbR.move(127);
        Misc::cdrift(-40,-40,600,true);
    }
    void intake(){
        while(1){
            if(TaskHandler::dIntake){
                if(!Misc::intakeR){
                    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) { Motor::intake.move(127); }
                    else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) { Motor::intake.move(-127); }
                    else{ Motor::intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST); Motor::intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST); Motor::intake.brake(); }
                }
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
        bool isUp = false;
        while(1){
            if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN) ) { Misc::togglePiston(Piston::release, b_hang);  Lift::setState(550); } 
            if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) { 
                Misc::togglePiston(Piston::pto, b_pto); 
                Misc::brakeState = (Misc::brakeState == pros::E_MOTOR_BRAKE_HOLD) ? pros::E_MOTOR_BRAKE_COAST : pros::E_MOTOR_BRAKE_HOLD;
            }
            if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) { Hang::release(); }
            pros::delay(Misc::DELAY);
        }
    }
    void release(){
        Lift::setState(1500);
        Motor::intake.move(-60);
        Piston::lightsaberL.set_value(true);
        pros::delay(200);
        Piston::lightsaberL.set_value(false);
        pros::delay(300);
        Motor::intake.move(0);
        Lift::setState(0);
        // Lift::setState(1500);
        // Piston::lightsaberL.set_value(true);
        // pros::delay(200);
        // Piston::lightsaberL.set_value(false);
        // pros::delay(300);
        // Lift::setState(0);
    }
} // namespace Driver
// <-------------------------------------------------------------- Auton ----------------------------------------------------------->
namespace Screen {
    void update() {
        controller.clear();  
        pros::delay(500);
        controller.set_text(0, 0, "1: " + std::to_string(Sensor::o_colorSort.get_proximity()));
        // controller.set_text(0, 0, "Pos: " + std::to_string(Motor::lbL.get_position()));
        // controller.set_text(0, 0, "Dist: " + std::to_string(leftMotors.get_position()));
        // controller.set_text(0, 0, "Run Time: " + std::to_string(pros::millis() / 1000) + "s");
        // controller.set_text(0, 0, "X: " + std::to_string(chassis.getPose().x) + "\nY: " + std::to_string(chassis.getPose().y));
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
    {"Red Negative Quals", Auton::Red::Qual::negNormal2},
    {"Blue Negative Quals", Auton::Blue::Qual::negNormal2},
    {"Red Positive Quals", Auton::Red::Qual::pos},
    {"Blue Positive Quals", Auton::Blue::Qual::pos},
    {"Red Solo", Auton::Red::Qual::solo},
    {"Blue Solo", Auton::Blue::Qual::solo3},
    {"Skills", Auton::Skills::main}
};

void autonSwitch() {
        pros::delay(Misc::DELAY);
        if (Sensor::autonSwitch.get_new_press()) { Auton::state++; if (Auton::state >= autonRoutines.size()) Auton::state = 0; }
        pros::lcd::set_text(4, autonRoutines[Auton::state].first);
}

Color::colorVals getColor(bool colorValV3) { return colorValV3 ? Color::colorVals::RED : Color::colorVals::BLUE; } // Sort ? Red : Blue

LV_IMG_DECLARE(tdbg);
LV_IMG_DECLARE(logo);

lv_obj_t * sbg = lv_img_create(lv_scr_act());
lv_obj_t * slogo = lv_img_create(lv_scr_act());

// <------------------------------------------------------------ Initialize --------------------------------------------------------------->
void initialize() {
    // pros::Task t_Select(autonSelectSwitch);

    // pros::lcd::initialize();
    // chassis.setPose(0, 0, 0);
    // chassis.calibrate(); 
    Motor::lbL.set_zero_position(0.0);
    Motor::lbR.set_zero_position(0.0);
    // Sensor::o_colorSort.set_led_pwm(100);
    // Sensor::o_colorSort.set_integration_time(5);
    // Motor::intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    
    // controller.clear();
    // lv_img_set_src(sbg, &tdbg);
	// lv_obj_set_pos(sbg,0,0);
	// lv_img_set_src(slogo, &logo);
	// lv_obj_set_pos(slogo,105,-15);
    set_up();
    pros::Task LVGL_upd([&]() {
        screen_upd();
    });
    TaskHandler::lbD =true;

    // pros::Task screenTask([&]() {
    //     while (1) {
    //         // pros::lcd::print(3, "Pos: %d", Sensor::lbR.get_position());
    //         // pros::lcd::print(3, "Pos: %f", Motor::lb.get_position());
    //         pros::lcd::print(0, "X: %f", chassis.getPose().x);
    //         pros::lcd::print(1, "Y: %f", chassis.getPose().y);
    //         pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
    //         pros::delay(50);
    //     }
    // });

    pros::Task autonSelect([]{ while(1){ if(TaskHandler::autonSelect) autonSwitch(); pros::delay(Misc::DELAY); }});
    pros::Task liftC([]{ while (1) { if(TaskHandler::lbD) Lift::lift(); pros::delay(Misc::DELAY); }});
    // pros::Task screenC([]{ while (1) { Screen::update(); pros::delay(100); }});
}
void disabled() {}
void competition_initialize() {}
ASSET(example_txt); // PP

// <------------------------------------------------------------- Autonomous ------------------------------------------------------------->
void autonomous() {
    // Color::state = Color::colorVals::BLUE;
    // TaskHandler::antiJam = true;
    // pros::Task sorterC([&](){ while(1) { Color::colorSort(Color::state);  pros::delay(5); }});
    // pros::Task toPosC([&](){ while(1) { Color::toPos(Color::colorConvertor(Color::state)); pros::delay(5); }});
    // pros::Task antiJam([&](){ while(1) { Jam::antiJam(); pros::delay(Misc::DELAY); }});
    // Sensor::o_colorSort.set_led_pwm(100);
    // Motor::intake.move(127);
    // Piston::mogo.set_value(true);
    // Auton::Red::Qual::negNormal2();
    // Auton::Red::Elim::negCloseWall2();
    // Auton::Red::Qual::solo3();
    // Auton::Red::Qual::pos();
    // Auton::Skills::main();

    // chassis.turnToHeading(0,900,{.maxSpeed = 100,.minSpeed = 10,.earlyExitRange = 1});
    // chassis.waitUntilDone();
    Motor::intake.move(0);
    TaskHandler::antiJam = false;
    Piston::release.set_value(true);
    Lift::setState(550);
    Misc::cdrift(-55,-55,1000);
    
    Piston::pto.set_value(true);
    Misc::brakeState = pros::E_MOTOR_BRAKE_COAST;

    TaskHandler::lbD = false;
    Motor::lbL.move(-127);
    Motor::lbR.move(-127);
    pros::delay(175);
    Motor::lbL.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    Motor::lbR.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    Motor::lbL.brake();
    Motor::lbR.brake();
    
    Hang::pull();
    Hang::release();
    Hang::pull();
    Hang::release();
    Hang::pull();
    Hang::release();
    Motor::lbL.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    Motor::lbR.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    Motor::intake.move(127);
    pros::delay(10000000);
    (Auton::state < autonRoutines.size()) ? autonRoutines[Auton::state].second() : Auton::Test::main();
}

// <--------------------------------------------------------------- Driver --------------------------------------------------------------->
void opcontrol() {
    // pros::Task sorterC([&](){ while(1) { if(TaskHandler::colorSort) Color::colorSort(Color::colorVals::RED);  pros::delay(Misc::DELAY);}});
    // TaskHandler::autonSelect = true;
    // pros::Task sorterTask([&](){ Color::colorSort(Color::colorVals::RED); pros::delay(Misc::DELAY); });
    // pros::Task sorterTask([](){ Color::colorSort(Color::colorVals::BLUE); });
    // static pros::Task sorterTask([](){ Color::colorSort(Color::colorConvertor(Color::state)); });
    // pros::Task toPosC([&](){ while(1) { if(TaskHandler::dIntake) Driver::intake(); pros::delay(Misc::DELAY); }});
    // Motor::intake.move(127);
    pros::Task intakeTask(Driver::intake);
    pros::Task driverTask(Driver::joystick);
    pros::Task pistonTask(Driver::piston);
    pros::Task hangTask(Driver::hang);
    TaskHandler::antiJam = false;
    TaskHandler::colorSort = false;
    // pros::Task startTask(Driver::release);
	leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
	rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    Motor::lbL.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    Motor::lbR.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    Lift::setState(0);
    // pros::lcd::clear();
    // lv_img_set_src(sbg, &tdbg);
	// lv_obj_set_pos(sbg,0,0);
	// lv_img_set_src(slogo, &logo);
	// lv_obj_set_pos(slogo,105,-15);
    while(1) {
        // Sensor::o_colorSort.set_led_pwm(100);
        // Color::colorSort(Color::colorVals::BLUE);
        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            TaskHandler::isDriver = false;
            Misc::cdrift(30,30,200,true);
            TaskHandler::isDriver = true;
        }
        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) Driver::release();
        pros::delay(Misc::DELAY);
    }
}