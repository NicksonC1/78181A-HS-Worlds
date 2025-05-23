# 3 - Driver Control

```{tip}
View the [example project](https://github.com/genesis/genesis/blob/stable/src/main.cpp) if you need more context for setup
```

## Introduction
In this tutorial, we will be learning how to program the drivetrain to allow for controller joystick inputs.

## The controller
The controller has 2 joysticks. A `left` one and a `right` one. Each joystick has 2 axes, an `x` axis and a `y` axis. We will be using these axes to control the drivetrain.

## Tank Drive
Tank drive is a simple method of controlling the drivetrain. You give it the power for the left wheels and the power for the right wheels. In this example, we will use the `left y` axis and the `right y` axis.

```cpp
pros::Controller controller(pros::E_CONTROLLER_MASTER);

void opcontrol() {
    // loop forever
    while (true) {
        // get left y and right y positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        // move the robot
        chassis.tank(leftY, rightY);

        // delay to save resources
        pros::delay(25);
    }
}
```

## Arcade Drive
Arcade drive is the most popular form of controlling the robot. In arcade control, we give the robot a forwards/backwards speed and a turning speed. Below are 2 examples: single stick arcade and double stick arcade.

### Single Stick Arcade

```cpp
pros::Controller controller(pros::E_CONTROLLER_MASTER);

void opcontrol() {
    // loop forever
    while (true) {
        // get left y and right x positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int leftX = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);

        // move the robot
        chassis.arcade(leftY, leftX);

        // delay to save resources
        pros::delay(25);
    }
}
```

### Double Stick Arcade

```cpp
pros::Controller controller(pros::E_CONTROLLER_MASTER);

void opcontrol() {
    // loop forever
    while (true) {
        // get left y and right x positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
        chassis.arcade(leftY, rightX);

        // delay to save resources
        pros::delay(25);
    }
}
```

### Throttle/Steer priority

```{info}
This section is optional and is not needed to control the robot
```

When we combine throttle and steering inputs, we can sometimes get a value over the motors' max voltage. 
For example, if the steering input and throttle input are both maxed out at 127, then the power supplied to the left drive will be 0, and the right drive will get 254, which will be rounded down to 127. 
This means the drive won't move forward at full speed, nor will it turn at full speed. 
This can be undesired behavior, as you may want steering to be consistent, no matter what throttle you provide.

Luckily, genesis provides you the authority to choose your desired behavior through the `desaturateBias` param.
`desaturateBias` has a range of 0 to 1, and only has an effect if motor output would be above 127.
If set to 0, steering will be reduced until the motor output is below 127, leaving throttle alone, and vice versa for a value 1.
The default is 0.5, where steering and throttle have the same priority. 
See the code block below:

```cpp
pros::Controller controller(pros::E_CONTROLLER_MASTER);

void opcontrol() {
    // loop forever
    while (true) {
        // get left y and right x positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int leftX = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);

        // move the robot
        // prioritize steering slightly
        chassis.arcade(leftY, leftX, false, 0.75);

        // delay to save resources
        pros::delay(25);
    }
}
```

## Curvature Drive
Curvature drive is a lesser-know, yet powerful, method. We give the robot a forwards/backwards speed, and the curvature of an arc. The greater the curvature, the more the robot turns. Its similar to arcade but performs better when turning. Below is an example of single stick and double stick curvature drive:

### Single Stick Curvature

```cpp
pros::Controller controller(pros::E_CONTROLLER_MASTER);

void opcontrol() {
    // loop forever
    while (true) {
        // get left y and right x positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int leftX = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);

        // move the robot
        chassis.curvature(leftY, leftX);

        // delay to save resources
        pros::delay(25);
    }
}
```

### Double Stick Curvature

```cpp
pros::Controller controller(pros::E_CONTROLLER_MASTER);

void opcontrol() {
    // loop forever
    while (true) {
        // get left y and right x positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
        chassis.curvature(leftY, rightX);

        // delay to save resources
        pros::delay(25);
    }
}
```

## But which do I choose?
There is no right answer to this question. The driver should use whichever control method they feel most comfortable with.

## Input Scaling

```{info}
This section is optional and is not needed to control the robot
```

```{info}
Tank control only uses the throttle curve, it does not use the steer curve
```

```{seealso}
[Detailed explanation in Vex Forum](https://www.vexforum.com/t/expo-drive-genesiss-implementation/123337)
```

Making precise movements is difficult. If only there was a way make it less sensitive, but not limit the maximum speed. Well, there is a way, and it's called input scaling.

Instead of the regular linear relationship between controller input and drivetrain output, input scaling is an exponential relationship to make small movements less sensitive in exchange for making fast movements more sensitive. Below in an image of this relationship:

```{image} ../assets/3_driver_control/curve.jpeg
:align: center
:height: 400
```

### Code

```cpp
// input curve for throttle input during driver control
genesis::ExpoDriveCurve throttle_curve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
genesis::ExpoDriveCurve steer_curve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
genesis::Chassis chassis(drivetrain,
                        lateral_controller,
                        angular_controller,
                        sensors,
                        &throttle_curve, 
                        &steer_curve
);
```


## Conclusion
That's all for driver control. We will be covering autonomous motion and tuning in the next tutorial.
