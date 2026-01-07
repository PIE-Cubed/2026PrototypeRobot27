package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class Controls {
    XboxController controller;
    ZorroController driveController;

    public static final double DRIVE_CONTROLLER_DEADZONE = 0.01;

    public Controls() {
        controller = new XboxController(1);
        driveController = new ZorroController(0);
    }

    // Zorro Controller

    public double getForwardPower() {
        // Multiplied by negative 1 because the controller gives the opposite value on the y axis then it should.
        //double power = -1 * controller.getLeftY();
        double power = -1 * driveController.getLeftY();

        //System.out.println(enablePrecisionDrive());
        power = Math.pow(power, 3);
        
        if (enablePrecisionDrive()) {
            power *= 0.231;
        }
        //System.out.println(power);
        return power;
    }

    public double getStrafePower() {
        //double power = controller.getLeftX();
        double power = driveController.getLeftX();

        //power = Math.pow(power, 3);

        // For Zorro:
        power = Math.pow(power, 3) * -1;
        
        if (enablePrecisionDrive()) {
            power *= 0.231;
        }
        return power;
    }

    public double getRotatePower() {
        //double power = controller.getRightX();
        double power = driveController.getRightX();

        power = Math.pow(power, 3);

        //System.out.println(power);
        return power;
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

    /*********************************************/
    /*                                           */
    /*              Xbox Controller              */
    /*                                           */
    /*********************************************/

    public boolean resetGyro() {
        //return controller.getPOV() == 0;
        //return driveController.getAButtonPressed() == true;
        return controller.getRightStickButtonPressed() == true;
        //return false;
    }
}