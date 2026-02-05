package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class Controls {

    XboxController manipController;
    ZorroController driveController;

    public static final double DRIVE_CONTROLLER_DEADZONE = 0.01;

    public Controls() {
        manipController = new XboxController(1);
        driveController = new ZorroController(0);
    }

    // Zorro Controller

    public double getForwardPowerFwdPositive() {
        double powerFwdPos = driveController.getLeftY(); // The Zorro controller returns forward positive.

        powerFwdPos = Math.pow(powerFwdPos, 3); // Raise to the power of 3 to help with small movements.

        if (enablePrecisionDrive()) { // Reduce output if precisionDrive is enabled.
            powerFwdPos *= 0.231;
        }

        return powerFwdPos; // We do not need to invert here as the Zorro already returns forward positive.
    }

    public double getStrafePowerLeftPositive() {
        double powerRightPos = driveController.getLeftX(); // The Zorro controller returns right positive.

        powerRightPos = Math.pow(powerRightPos, 3); // Raise to the power of 3 to help with small movements.

        if (enablePrecisionDrive()) { // Reduce output if precisionDrive is enabled.
            powerRightPos *= 0.231;
        }

        return powerRightPos * -1; // We want left positive, so it is necessary to invert here.
    }

    public double getRotatePowerCcwPositive() {
        double powerCwPos = driveController.getRightX(); // The Zorro controller returns right (clockwise) positive.

        powerCwPos = Math.pow(powerCwPos, 3); // Raise to the power of 3 to help with small movements.

        return powerCwPos * -1; // We want left (counter-clockwise) positive, so it is necessary to invert here.
    }

    public double getRightY() {
        return driveController.getRightY(); // The Zorro controller returns forward positive.
    }

    public boolean enablePrecisionDrive() {
        //return controller.getRightTriggerAxis() > 0.05;
        return driveController.getFTwoPosSwitch() == true;
    }

    public boolean getFieldDrive() {
        return !driveController.getETwoPosSwitch();
    }

    public boolean getWheelLock() {
        if (driveController.getBThreePosSwitch() == 1) {
            return true;
        } else {
            return false;
        }
    }

    public boolean resetGyro() {
        //return controller.getPOV() == 0;
        //return driveController.getAButtonPressed() == true;
        return driveController.getHButton() == true;
        //return false;
    }

    /*********************************************/
    /*                                           */
    /*              Xbox Controller              */
    /*                                           */
    /*********************************************/

    public boolean getShootButton() {
        return manipController.getRightBumperButton();
    }

    public boolean getAButton() {
        return manipController.getAButton();
    }

    public double getLeftTrigger() {
        return manipController.getLeftTriggerAxis() * 0.8;
    }

    public double getRightTrigger() {
        return manipController.getRightTriggerAxis() * 0.8;
    }
}
