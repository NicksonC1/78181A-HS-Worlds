#pragma once

#include "main.h"
#include "pros/rtos.hpp"
#include "genesis/api.hpp"

extern pros::Controller controller;

extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;

namespace Motor{
    // extern pros::Motor intakeB;
    extern pros::Motor intakeT;
    extern pros::Motor lbL;
    extern pros::Motor lbR;
} // namespace Motor

namespace Sensor{
    // extern pros::Rotation lbR;
    extern pros::Distance hang;
    extern pros::Optical o_colorSort;
    // extern pros::Distance d_colorSort;
    extern pros::adi::DigitalIn autonSwitch;
} // namspace Sensor

namespace Piston{
    extern pros::adi::DigitalOut lightsaberL;
    extern pros::adi::DigitalOut saberclamp;
    extern pros::adi::DigitalOut mogo;
    extern pros::adi::DigitalOut pto;
    extern pros::adi::DigitalOut release;
} // namespace Piston

class CustomIMU : public pros::IMU {
    public:
        CustomIMU(int port, double scalar);
        virtual double get_rotation() const override;

    private:
        const int m_port;
        const double m_scalar;
};

// extern pros::Imu imu;
extern CustomIMU s_imu;
extern pros::Rotation horizontalEnc;
extern pros::Rotation verticalEnc;

extern genesis::TrackingWheel horizontal;
extern genesis::TrackingWheel vertical;
extern genesis::Drivetrain drivetrain;
extern genesis::ControllerSettings linearController;
extern genesis::ControllerSettings angularController;
extern genesis::OdomSensors sensors;
extern genesis::ExpoDriveCurve throttleCurve;
extern genesis::ExpoDriveCurve steerCurve;
extern genesis::Chassis chassis;
