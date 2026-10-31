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
#include <mutex>

using namespace vex;

// A global instance of vex::brain used for printing to the V5 brain screen
brain robotBrain = brain();

// port settings
struct {
    int32_t frontLeftDrivePort, frontRightDrivePort, backLeftDrivePort, backRightDrivePort,
    intakePort, armPort, wristPort, auxiliaryPort;

    triport::port clawPort;
} portMapping = {
    // drive motors
    .frontLeftDrivePort = PORT2,
    .frontRightDrivePort = PORT3,
    .backLeftDrivePort = PORT1,
    .backRightDrivePort = PORT4,

    // other stuff
    .intakePort = PORT10,
    .armPort = PORT12,
    .wristPort = PORT11,
    .auxiliaryPort = PORT5,

    // three-wire ports
    .clawPort = robotBrain.ThreeWirePort.H,
};

// define your global instances of motors and other devices here
controller driverController = controller();

motor intakeMotor = motor(portMapping.intakePort, false);
motor armMotor = motor(portMapping.armPort, true);
motor wristMotor = motor(portMapping.wristPort, false);
motor auxiliaryMotor = motor(portMapping.auxiliaryPort, true);
digital_out clawSolenoid = digital_out(portMapping.clawPort);

// Calibration: start with the arm straight down and the claw horizontal.
// These are motor degrees per joint degree, including any external gearing.
// Negate a ratio if that motor's positive direction lowers its joint.
constexpr double armMotorDegreesPerJointDegree = 84.0 * 36 / 36.0;
constexpr double wristMotorDegreesPerJointDegree = 60.0 * 18 / 12.0;
constexpr double wristLevelingProportionalGain = 1;
constexpr double wristLevelingMaxSpeedPercent = 35.0;
bool wristLevelingEnabled = false;

bool intakeEnabled = false;
bool intakeReversed = false;
vex::mutex intakeControlMutex;

double driveSpeedMultiplier = 0.25;

bool clawClosed = false;

bool pickupRunning = false;
vex::mutex pickupControlMutex;

struct {
    motor frontLeft, frontRight, backLeft, backRight;
} driveMotors = {
    .frontLeft = motor(portMapping.frontLeftDrivePort, false),
    .frontRight = motor(portMapping.frontRightDrivePort, true),
    .backLeft = motor(portMapping.backLeftDrivePort, false),
    .backRight = motor(portMapping.backRightDrivePort, true),
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
    double wheelCircumferenceInches = 4 * M_PI;
    double wheelPathCircumferenceInches = 14.75 * M_PI;

    // average wheel positions
    double averageWheelPositionDegrees = (driveMotors.frontLeft.position(deg) - driveMotors.frontRight.position(deg) + driveMotors.backLeft.position(deg) - driveMotors.backRight.position(deg)) / 4;

    double wheelTravelInches = averageWheelPositionDegrees * (wheelCircumferenceInches); // distance wheel has moved along the ground
    double robotTurnFraction = wheelTravelInches / wheelPathCircumferenceInches; // amount turned as a decimal 0 to 1
    double robotHeadingDegrees = robotTurnFraction * 360; // convert the turn fraction to degrees
    return robotHeadingDegrees;
}

void toggleDriveSpeed() {
    if (driveSpeedMultiplier == 0.25) {
        driveSpeedMultiplier = 0.75;
    } else if (driveSpeedMultiplier == 0.75) {
        driveSpeedMultiplier = 0.25;
    } else {
        driveSpeedMultiplier = 0.25;
    }
}

void drive(double forwardInputPercent, double strafeInputPercent, double rotationInputPercent) {
    driveMotors.frontLeft.setVelocity((forwardInputPercent + strafeInputPercent + rotationInputPercent) * driveSpeedMultiplier, percent);
    driveMotors.frontRight.setVelocity((forwardInputPercent - strafeInputPercent - rotationInputPercent) * driveSpeedMultiplier, percent);
    driveMotors.backLeft.setVelocity((forwardInputPercent - strafeInputPercent + rotationInputPercent) * driveSpeedMultiplier, percent);
    driveMotors.backRight.setVelocity((forwardInputPercent + strafeInputPercent - rotationInputPercent) * driveSpeedMultiplier, percent);
}

void toggleIntake() {
    std::lock_guard<vex::mutex> intakeControlLock(intakeControlMutex);
    intakeEnabled = !intakeEnabled;
    intakeReversed = false;

    if (intakeEnabled) {
        intakeMotor.spin(forward, 100, percent);
    } else {
        intakeMotor.stop();
    }
}

void reverseIntake() {
    std::lock_guard<vex::mutex> intakeControlLock(intakeControlMutex);
    if (!intakeEnabled) return;

    intakeReversed = !intakeReversed;
    intakeMotor.spin(intakeReversed ? reverse : forward, 100, percent);
}

void setArmVelocity() {
    std::lock_guard<vex::mutex> pickupControlLock(pickupControlMutex);
    if (pickupRunning) return;

    if (!(driverController.ButtonL1.pressing() ^ driverController.ButtonL2.pressing())) {
        armMotor.stop();
        armMotor.setVelocity(0, percent);
        return;
    }
    if (driverController.ButtonL1.pressing()) {
        armMotor.spin(forward);
        armMotor.setVelocity(20, percent);
        return;
    }
    if (driverController.ButtonL2.pressing()) {
        armMotor.spin(forward);
        armMotor.setVelocity(-20, percent);
        return;
    }
}

void setWristVelocity() {
    std::lock_guard<vex::mutex> pickupControlLock(pickupControlMutex);
    if (pickupRunning) return;

    // Manual wrist input cancels automatic leveling.
    if (driverController.ButtonUp.pressing() || driverController.ButtonDown.pressing())
        wristLevelingEnabled = false;

    if (wristLevelingEnabled) {
        // Encoders measure changes from the startup pose. Cancel the arm's
        // rotation to preserve the claw's horizontal startup orientation.
        const double armAngleChangeDegrees = armMotor.position(degrees) / armMotorDegreesPerJointDegree;
        const double targetWristPositionDegrees = -armAngleChangeDegrees * wristMotorDegreesPerJointDegree;
        const double wristPositionErrorDegrees = targetWristPositionDegrees - wristMotor.position(degrees);
        double wristSpeedPercent = wristPositionErrorDegrees * wristLevelingProportionalGain;
        if (wristSpeedPercent > wristLevelingMaxSpeedPercent) wristSpeedPercent = wristLevelingMaxSpeedPercent;
        if (wristSpeedPercent < -wristLevelingMaxSpeedPercent) wristSpeedPercent = -wristLevelingMaxSpeedPercent;

        if (fabs(wristPositionErrorDegrees) < 0.5)
            wristMotor.stop(hold);
        else
            wristMotor.spin(forward, wristSpeedPercent, percent);
        return;
    }

    if (!(driverController.ButtonUp.pressing() ^ driverController.ButtonDown.pressing())) {
        wristMotor.stop();
        wristMotor.setVelocity(0, percent);
        return;
    }
    if (driverController.ButtonUp.pressing()) {
        wristMotor.spin(forward);
        wristMotor.setVelocity(25, percent);
        return;
    }
    if (driverController.ButtonDown.pressing()) {
        wristMotor.spin(forward);
        wristMotor.setVelocity(-25, percent);
        return;
    }
}

void toggleWristLeveling() {
    std::lock_guard<vex::mutex> pickupControlLock(pickupControlMutex);
    if (pickupRunning) return;

    wristLevelingEnabled = !wristLevelingEnabled;
}

void toggleClaw() {
    std::lock_guard<vex::mutex> pickupControlLock(pickupControlMutex);
    if (pickupRunning) return;

    clawSolenoid.set(clawClosed = !clawClosed);
}

void toggleAuxiliaryMotor() {
    static bool auxiliaryMotorEnabled = false;
    auxiliaryMotor.setVelocity(25 * (auxiliaryMotorEnabled = !auxiliaryMotorEnabled), percent);
}

// Manual joint input cancels pickup; normal control resumes after cleanup.
bool pickupCancelled() {
    return driverController.ButtonL1.pressing() || driverController.ButtonL2.pressing()
        || driverController.ButtonUp.pressing() || driverController.ButtonDown.pressing();
}

bool waitForPickupMotion() {
    while (!armMotor.isDone() || !wristMotor.isDone()) {
        if (pickupCancelled()) return false;
        wait(5, msec);
    }
    return !pickupCancelled();
}

bool pickupPause() {
    for (int elapsedMilliseconds = 0; elapsedMilliseconds < 125; elapsedMilliseconds += 5) {
        if (pickupCancelled()) return false;
        wait(5, msec);
    }
    return !pickupCancelled();
}

void pickupFromIntake() {
    // Ignore another pickup request until this sequence has fully exited.
    {
        std::lock_guard<vex::mutex> pickupControlLock(pickupControlMutex);
        if (pickupRunning) return;
        pickupRunning = true;
        wristLevelingEnabled = false;
        armMotor.stop(hold);
        wristMotor.stop(hold);
    }

    // Early exits still pass through the cleanup below.
    auto runPickupSequence = []() {
        if (pickupCancelled()) return;

        clawSolenoid.set(true);
        clawClosed = true;
        
        wristMotor.spinToPosition(12.0, degrees, 35, velocityUnits::pct, false);
        if (!waitForPickupMotion() || !pickupPause()) return;
        armMotor.spinToPosition(-300, degrees, 35, velocityUnits::pct, false); // -188.8
        if (!waitForPickupMotion()) return;

        clawSolenoid.set(false);
        clawClosed = false;

        armMotor.spinToPosition(-200, degrees, 35, velocityUnits::pct, false);
        if (!waitForPickupMotion()) return;
        wristMotor.spinToPosition(-275.0, degrees, 35, velocityUnits::pct, false);
        if (!waitForPickupMotion() || !pickupPause()) return;

        clawSolenoid.set(true);
        clawClosed = true;
        if (!pickupPause()) return;

        wristMotor.spinToPosition(56.8, degrees, 35, velocityUnits::pct, false);
        if (!waitForPickupMotion() || !pickupPause()) return;

        armMotor.spinToPosition(54.4, degrees, 35, velocityUnits::pct, false);
        // wristMotor.spinToPosition(-114.0, degrees, 35, velocityUnits::pct, false);
        waitForPickupMotion();
    };
    runPickupSequence();

    armMotor.stop(hold);
    wristMotor.stop(hold);
    std::lock_guard<vex::mutex> pickupControlLock(pickupControlMutex);
    pickupRunning = false;
}

int main() {
    driveMotors.frontLeft.spin(forward);
    driveMotors.frontLeft.setStopping(brake);
    driveMotors.frontRight.spin(forward);
    driveMotors.frontRight.setStopping(brake);
    driveMotors.backLeft.spin(forward);
    driveMotors.backLeft.setStopping(brake);
    driveMotors.backRight.spin(forward);
    driveMotors.backRight.setStopping(brake);

    armMotor.setStopping(hold);
    // armMotor.setStopping(coast);
    armMotor.stop();
    armMotor.setMaxTorque(100, percent);
    wristMotor.setStopping(hold);
    // wristMotor.setStopping(coast);
    wristMotor.stop();
    wristMotor.setMaxTorque(100, percent);
    // Zero at startup: arm straight down, claw horizontal.
    armMotor.setPosition(0, degrees);
    wristMotor.setPosition(0, degrees);
    intakeMotor.stop();
    auxiliaryMotor.setVelocity(0, percent);
    auxiliaryMotor.spin(forward);

    driverController.ButtonA.pressed(toggleIntake);
    driverController.ButtonR1.pressed(toggleClaw);
    driverController.ButtonR2.pressed(toggleAuxiliaryMotor);
    driverController.ButtonRight.pressed(toggleWristLeveling);
    driverController.ButtonB.pressed(reverseIntake);
    driverController.ButtonY.pressed(toggleDriveSpeed);
    driverController.ButtonLeft.pressed(pickupFromIntake);
   
    while(this_thread::sleep_for(10), 1) {
        drive(driverController.Axis3.position(), driverController.Axis4.position(), driverController.Axis1.position());
        setArmVelocity();
        setWristVelocity();
        robotBrain.Screen.printAt(10, 50, "Arm Pos: %f", armMotor.position(degrees));
        robotBrain.Screen.printAt(10, 65, "Wrist Pos: %f", wristMotor.position(degrees));
    }
}
