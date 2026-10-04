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
    intake, arm;

    triport::port claw;
} portMapping = {
    // drive motors
    .driveFrontLeft = PORT1,
    .driveFrontRight = PORT2,
    .driveBackLeft = PORT3,
    .driveBackRight = PORT4,

    // other stuff
    .intake = PORT5,
    .arm = PORT6,

    // three-wire ports
    .claw = Brain.ThreeWirePort.A,
};

// define your global instances of motors and other devices here
controller controller_main = controller();

motor intake = motor(portMapping.intake, false);
motor arm = motor(portMapping.arm, true);
pneumatics claw = pneumatics(portMapping.claw);

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
    if (!(controller_main.ButtonL1.pressing() ^ controller_main.ButtonL1.pressing())) {
        arm.setVelocity(0, percent);
        return;
    }
    if (controller_main.ButtonL1.pressing()) {
        arm.setVelocity(100, percent);
        return;
    }
    if (controller_main.ButtonL2.pressing()) {
        arm.setVelocity(-100, percent);
        return;
    }
}

void openClaw() {
    claw.open();
}

void closeClaw() {
    claw.close();
}

int main() {
    Brain.Screen.printAt( 10, 50, "Hello V5" );
    x_drive.fl.spin(forward);
    x_drive.fr.spin(forward);
    x_drive.bl.spin(forward);
    x_drive.br.spin(forward);

    arm.setStopping(hold);

    controller_main.ButtonA.pressed(toggleIntake);
    controller_main.ButtonR1.pressed(openClaw);
    controller_main.ButtonR2.pressed(closeClaw);
   
    while(1) {
        drive(controller_main.Axis3.position(), controller_main.Axis4.position(), controller_main.Axis1.position());
        setArmVelocity();
        
        // Allow other tasks to run
        this_thread::sleep_for(10);
    }
}
