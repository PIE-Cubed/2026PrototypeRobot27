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
        double powerFwdPos = driveController.getLeftY();

        powerFwdPos = Math.pow(powerFwdPos, 3);
        
        if (enablePrecisionDrive()) {
            powerFwdPos *= 0.231;
        }

        return powerFwdPos;
    }


    public double getStrafePowerLeftPositive() {
        double powerRightPos = driveController.getLeftX();

        powerRightPos = Math.pow(powerRightPos, 3);
        
        if (enablePrecisionDrive()) {
            powerRightPos *= 0.231;
        }
        return powerRightPos * -1;
    }


    public double getRotatePowerCcwPositive() {
        double powerCwPos = driveController.getRightX();

        powerCwPos = Math.pow(powerCwPos, 3);

        return powerCwPos * -1;
    }


    public double getRightY() {
        return driveController.getRightY();
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
        } 
        else {
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
}