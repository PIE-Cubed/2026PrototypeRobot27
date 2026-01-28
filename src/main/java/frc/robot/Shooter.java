package frc.robot;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.FieldConstants;
import frc.robot.util.AllianceUtil;

public class Shooter {
    private final int FLYWHEEL_MOTOR_ID = 20;
    private final int BACKSPIN_MOTOR_ID = 21;

    // backspin motor runs slower than flywheel motor to generate spin
    private static final double BACKSPIN_POWER_DELTA = .1;
    private static final double RPM_PER_VOLT = 550;
    private static final double RPM_TO_POWER = 6175;

    // PID Values
    private final double BACKSPIN_P = 0.007; // 0.0055
    private final double BACKSPIN_I = 0.0;
    private final double BACKSPIN_D = 0.0;
    private final double BACKSPIN_TOLERANCE = 25;

    private final double FLYWHEEL_P = 0.0055;
    private final double FLYWHEEL_I = 0.0;
    private final double FLYWHEEL_D = 0.0;
    private final double FLYWHEEL_TOLERANCE = 25;

    public static enum ShooterTarget {
        HUB,
        PASS
    }

    private SparkBase flywheelMotor = new SparkFlex(FLYWHEEL_MOTOR_ID, MotorType.kBrushless);
    private SparkBaseConfig flywheelMotorConfig = new SparkFlexConfig();

    private SparkBase backspinMotor = new SparkFlex(BACKSPIN_MOTOR_ID, MotorType.kBrushless);
    private SparkBaseConfig backspinMotorConfig = new SparkFlexConfig();

    private RelativeEncoder flywheelMotorEncoder;
    private RelativeEncoder backspinMotorEncoder;

    private PIDController flywheelPIDController;
    private PIDController backspinPIDController;

    private int flywheelAtSetpointCount = 0;
    private int backspinAtSetpointCount = 0;

    private Pose2d targetPose     = new Pose2d();
    public static double targetDistance = 0;

    public Shooter() {
        flywheelMotorConfig.idleMode(IdleMode.kCoast)
                           .inverted(false)
                           .smartCurrentLimit(Robot.NEO_CURRENT_LIMIT);

        flywheelMotor.configure(flywheelMotorConfig, 
                                ResetMode.kNoResetSafeParameters, 
                                PersistMode.kPersistParameters);
        
        backspinMotorConfig.idleMode(IdleMode.kCoast)
                           .inverted(true)
                           .smartCurrentLimit(Robot.NEO_CURRENT_LIMIT);

        backspinMotor.configure(backspinMotorConfig, 
                                ResetMode.kNoResetSafeParameters, 
                                PersistMode.kPersistParameters);
                                

        flywheelMotorEncoder = flywheelMotor.getEncoder();
        backspinMotorEncoder = backspinMotor.getEncoder();

        flywheelPIDController = new PIDController(FLYWHEEL_P, FLYWHEEL_I, FLYWHEEL_D);
        flywheelPIDController.setTolerance(FLYWHEEL_TOLERANCE);

        backspinPIDController = new PIDController(BACKSPIN_P, BACKSPIN_I, BACKSPIN_D);
        backspinPIDController.setTolerance(BACKSPIN_TOLERANCE);
    }

    

    public int spinUp(double distance) {

        return Robot.CONT;
    }

    public void setMotorPowers(double flywheelPower, double backspinPower) {
        flywheelMotor.set(flywheelPower);
        backspinMotor.set(backspinPower);
    }

    public int setFlywheelRPM(double targetRPM) {
        double voltage = flywheelPIDController.calculate(flywheelMotorEncoder.getVelocity(), targetRPM);

        flywheelMotor.setVoltage(voltage);

        if (flywheelPIDController.atSetpoint()) {
            flywheelAtSetpointCount += 1;
        }
        else {
            flywheelAtSetpointCount = 0;
        }

        if (flywheelAtSetpointCount >= 3) {
            return Robot.DONE;
        }

        return Robot.CONT;
    }

    public int setBackspinRPM(double targetRPM) {
        double voltage = backspinPIDController.calculate(backspinMotorEncoder.getVelocity(), targetRPM);

        backspinMotor.setVoltage(voltage);

        if (backspinPIDController.atSetpoint()) {
            backspinAtSetpointCount += 1;
        }
        else {
            backspinAtSetpointCount = 0;
        }

        if (backspinAtSetpointCount >= 3) {
            return Robot.DONE;
        }

        return Robot.CONT;
    }
}

        /* FOR MOTOR SET FUCNTION
         * .1 power = 606 RPM
         * .2 power = 1240 RPM
         * .3 power = 1866 RPM
         * .4 power = 2500 RPM
         * .5 power = 3140 RPM
         * .6 power = 3783 RPM
         * .7 power =  4408 RPM
         * .8 power =  5012 RPM
         * .9 power = 5600 RPM
         * 1 power =  6175 RPM
         * 
         * MOTORSETVOLTAGE (More Stable than SET FUNCTOIN)
         * volts  RPM    Delta   RPM/Volts
         * 1      527            527
         * 2      1085   558     542.5
         * 3      1642   560     547.3
         * 4      2201   559     550.25
         * 5      2761   560     552.2
         * 6      3322   561     553.67
         * 7      3884   562     554.85
         * 8      4446   562     555.75
         * 9      5006   560     556.22
         * 10     5550   544     555
         * 11     6090   540     553.64
         * MAX 12 6155   065     512.92
         */

/*
// code in case we have a turret
public int aimAt(ShooterTarget target) {
    if (target == ShooterTarget.HUB) {
        if (AllianceUtil.isRedAlliance()) {
            targetPose = FieldConstants.hubRedAlliance;
        }
        else {
            targetPose = FieldConstants.hubBlueAlliance;
        }
    }
    else if (target == ShooterTarget.PASS) {
        if (AllianceUtil.isRedAlliance()) {
            targetPose = new Pose2d(15.75, Odometry.getPose().getY(), new Rotation2d(0));
        }
        else {
            targetPose = new Pose2d(1, Odometry.getPose().getY(), new Rotation2d(0));
        }
    }

    // put turret pid control here

    return Robot.CONT;
}
*/