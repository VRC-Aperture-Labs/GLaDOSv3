/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       connoregan                                                */
/*    Created:      8/2/2026, 2:23:33 PM                                      */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
#include "math.h"

using namespace vex;

// A global instance of vex::brain used for printing to the V5 brain screen
brain Brain = brain();

// port settings
struct {
    int32_t driveFrontLeft, driveFrontRight, driveBackLeft, driveBackRight,
    intake, arm, wrist, toggleToggler;

    triport::port claw;
} portMapping = {
    // drive motors
    .driveFrontLeft = PORT2,
    .driveFrontRight = PORT3,
    .driveBackLeft = PORT1,
    .driveBackRight = PORT4,

    // other stuff
    .intake = PORT10,
    .arm = PORT12,
    .wrist = PORT11,
    .toggleToggler = PORT5,

    // three-wire ports
    .claw = Brain.ThreeWirePort.H,
};

// define your global instances of motors and other devices here
controller controller_main = controller();

motor intake = motor(portMapping.intake, false);
motor arm = motor(portMapping.arm, true);
motor wrist = motor(portMapping.wrist, false);
motor toggleToggler = motor(portMapping.toggleToggler, true);
digital_out claw = digital_out(portMapping.claw);

// Calibration: start with the arm straight down and the claw horizontal.
// These are motor degrees per joint degree, including any external gearing.
// Negate a ratio if that motor's positive direction lowers its joint.
constexpr double armMotorDegreesPerJointDegree = 84.0 * 36 / 36.0;
constexpr double wristMotorDegreesPerJointDegree = 60.0 * 18 / 12.0;
constexpr double wristLevelGain = 1;
constexpr double wristLevelMaxSpeed = 35.0;
bool wristLevelingEnabled = false;

struct {
    motor fl, fr, bl, br;
} x_drive = {
    .fl = motor(portMapping.driveFrontLeft, false),
    .fr = motor(portMapping.driveFrontRight, true),
    .bl = motor(portMapping.driveBackLeft, false),
    .br = motor(portMapping.driveBackRight, true),
};

// numbers for true north system:
// * = calculated
// wheel circle diameter 14 3/4 in.
// wheel circle circumference*: 14 3/4 * pi in.
// wheel diameter: 4 in.
// wheel circumference*: 4 * pi in.

double trueNorth() {
    // calculates delta degrees from true north to current position
    // positive return value means clockwise
    double wheelCircumference = 4 * M_PI;
    double wheelCircleCircumference = 14.75 * M_PI;

    // average wheel positions
    double pos = (x_drive.fl.position(deg) - x_drive.fr.position(deg) + x_drive.bl.position(deg) - x_drive.br.position(deg)) / 4;

    double wheelDistance = pos * (wheelCircumference); // distance wheel has moved along the ground
    double amountTurned = wheelDistance / wheelCircleCircumference; // amount turned as a decimal 0 to 1
    double degreesTurned = amountTurned * 360; // turn amountTurned into degrees
    return degreesTurned;
}

void drive(double forward, double right, double rotate) {
    x_drive.fl.setVelocity(forward + right + rotate, percent);
    x_drive.fr.setVelocity(forward - right - rotate, percent);
    x_drive.bl.setVelocity(forward - right + rotate, percent);
    x_drive.br.setVelocity(forward + right - rotate, percent);
}

void toggleIntake() {
    static bool enabled = false;

    if (enabled)
        intake.setVelocity(0, percent);
    else
        intake.setVelocity(100, percent);
    
    enabled = !enabled;
}

void setArmVelocity() {
    if (!(controller_main.ButtonL1.pressing() ^ controller_main.ButtonL2.pressing())) {
        arm.stop();
        arm.setVelocity(0, percent);
        return;
    }
    if (controller_main.ButtonL1.pressing()) {
        arm.spin(forward);
        arm.setVelocity(20, percent);
        return;
    }
    if (controller_main.ButtonL2.pressing()) {
        arm.spin(forward);
        arm.setVelocity(-20, percent);
        return;
    }
}

void setWristVelocity() {
    // Manual wrist input cancels automatic leveling.
    if (controller_main.ButtonUp.pressing() || controller_main.ButtonDown.pressing())
        wristLevelingEnabled = false;

    if (wristLevelingEnabled) {
        // Encoders measure changes from the startup pose. Cancel the arm's
        // rotation to preserve the claw's horizontal startup orientation.
        const double armAngleChange = arm.position(degrees) / armMotorDegreesPerJointDegree;
        const double target = -armAngleChange * wristMotorDegreesPerJointDegree;
        const double error = target - wrist.position(degrees);
        double speed = error * wristLevelGain;
        if (speed > wristLevelMaxSpeed) speed = wristLevelMaxSpeed;
        if (speed < -wristLevelMaxSpeed) speed = -wristLevelMaxSpeed;

        if (fabs(error) < 0.5)
            wrist.stop(hold);
        else
            wrist.spin(forward, speed, percent);
        return;
    }

    if (!(controller_main.ButtonUp.pressing() ^ controller_main.ButtonDown.pressing())) {
        wrist.stop();
        wrist.setVelocity(0, percent);
        return;
    }
    if (controller_main.ButtonUp.pressing()) {
        wrist.spin(forward);
        wrist.setVelocity(25, percent);
        return;
    }
    if (controller_main.ButtonDown.pressing()) {
        wrist.spin(forward);
        wrist.setVelocity(-25, percent);
        return;
    }
}

void toggleWristLeveling() {
    wristLevelingEnabled = !wristLevelingEnabled;
}

void toggleClaw() {
    static bool clawClosed = false;
    claw.set(clawClosed = !clawClosed);
}

void toggleToggleToggler() {
    static bool toggleTogglerEnabled = false;
    toggleToggler.setVelocity(25 * (toggleTogglerEnabled = !toggleTogglerEnabled), percent);
}

int main() {
    x_drive.fl.spin(forward);
    x_drive.fl.setStopping(brake);
    x_drive.fr.spin(forward);
    x_drive.fr.setStopping(brake);
    x_drive.bl.spin(forward);
    x_drive.bl.setStopping(brake);
    x_drive.br.spin(forward);
    x_drive.br.setStopping(brake);

    arm.setStopping(hold);
    arm.stop();
    arm.setMaxTorque(100, percent);
    wrist.setStopping(hold);
    wrist.stop();
    wrist.setMaxTorque(100, percent);
    // Zero at startup: arm straight down, claw horizontal.
    arm.setPosition(0, degrees);
    wrist.setPosition(0, degrees);
    intake.setVelocity(0, percent);
    intake.spin(forward);
    toggleToggler.setVelocity(0, percent);
    toggleToggler.spin(forward);

    controller_main.ButtonA.pressed(toggleIntake);
    controller_main.ButtonR1.pressed(toggleClaw);
    controller_main.ButtonR2.pressed(toggleToggleToggler);
    controller_main.ButtonRight.pressed(toggleWristLeveling);
   
    while(this_thread::sleep_for(10), 1) {
        drive(controller_main.Axis3.position(), controller_main.Axis4.position(), controller_main.Axis1.position());
        setArmVelocity();
        setWristVelocity();
        Brain.Screen.printAt(10, 50, "Arm Pos: %f", arm.position(degrees));
        Brain.Screen.printAt(10, 65, "Wrist Pos: %f", wrist.position(degrees));
    }
}
