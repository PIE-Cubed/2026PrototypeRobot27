package frc.robot;

import com.fasterxml.jackson.databind.util.RootNameLookup;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter {
    private SparkFlex       backspinMotor;
    private SparkBaseConfig backspinMotorConfig;

    private SparkFlex       flywheelMotor;
    private SparkBaseConfig flywheelMotorConfig;

    private RelativeEncoder backspinMotorEncoder;
    private RelativeEncoder flywheelMotorEncoder;

    private PIDController backspinPIDController;
    private PIDController flywheelPIDController;

    // Motor IDs
    private final int BACKSPIN_MOTOR_ID = 21;
    private final int FLYWHEEL_MOTOR_ID = 20;

    // PID Values
    private final double BACKSPIN_P         = 0.0055;
    private final double BACKSPIN_I         = 0.0;
    private final double BACKSPIN_D         = 0.0;
    private final double BACKSPIN_TOLERANCE = 50.0;

    private final double FLYWHEEL_P         = 0.0055;
    private final double FLYWHEEL_I         = 0.0;
    private final double FLYWHEEL_D         = 0.0;
    private final double FLYWHEEL_TOLERANCE = 50.0;

    private double prevShooterVelocity = 0;
    private double prevShooterVoltage  = 0;

    private double prevFlywheelRPM     = 0;
    private double prevFlywheelVoltage = 0;
    private double prevBackspinRPM     = 0;
    private double prevBackspinVoltage = 0;

    // top motor runs slower than bottom motor for backspin
    private static final double SHOOTER_MOTOR_DELTA = .1;
    private static final double VELOCITY_TO_VOLT_RATIO = 550;

    public Shooter() {
        backspinMotor       = new SparkFlex(BACKSPIN_MOTOR_ID, MotorType.kBrushless);
        backspinMotorConfig = new SparkFlexConfig();

        flywheelMotor       = new SparkFlex(FLYWHEEL_MOTOR_ID, MotorType.kBrushless);
        flywheelMotorConfig = new SparkFlexConfig();

        // TODO invert motor if needed
        backspinMotorConfig.idleMode(IdleMode.kCoast);
        backspinMotorConfig.inverted(true);

        flywheelMotorConfig.idleMode(IdleMode.kCoast);
        flywheelMotorConfig.inverted(false);

        backspinMotor.configure(backspinMotorConfig, 
                                ResetMode.kNoResetSafeParameters, 
                                PersistMode.kPersistParameters);

        flywheelMotor.configure(flywheelMotorConfig, 
                                ResetMode.kNoResetSafeParameters, 
                                PersistMode.kPersistParameters);

        backspinMotorEncoder = backspinMotor.getEncoder();
        flywheelMotorEncoder = flywheelMotor.getEncoder();

        backspinPIDController = new PIDController(BACKSPIN_P, BACKSPIN_I, BACKSPIN_D);
        backspinPIDController.setTolerance(BACKSPIN_TOLERANCE);

        flywheelPIDController = new PIDController(FLYWHEEL_P, FLYWHEEL_I, FLYWHEEL_D);
        flywheelPIDController.setTolerance(FLYWHEEL_TOLERANCE);
    }

    /**
     * voltage should be -12 to 12
     * @param flywheelVoltage
     * @param backspinVoltage
     */
    public void setMotorVoltages(double flywheelVoltage, double backspinVoltage) {
        setBottomMotorVoltage(flywheelVoltage);
        setTopMotorVoltage(backspinVoltage);
    }

    /**
     * voltage should be -12 to 12
     * @param voltage
     */
    public void setTopMotorVoltage(double voltage) {
        backspinMotor.setVoltage(voltage);
    }

    /**
     * voltage should be -12 to 12
     * @param voltage
     */
    public void setBottomMotorVoltage(double voltage) {
        flywheelMotor.setVoltage(voltage);
    }

    /*
     * maybe something to do eventually
     */
    public void setTargetArc(double height, double distance) {

    }

    /**
     * Sets voltage of the backspin motor according to a target RPM.
     * @param targetRPM Target RPM.
     * @return Robot.DONE if current RPM is within tolerance, otherwise Robot.CONT.
     */
    public int setBackspinTargetRPM(double targetRPM) {
        double currentRPM = backspinMotorEncoder.getVelocity();

        double voltage = backspinPIDController.calculate(currentRPM, targetRPM);

        if (backspinPIDController.atSetpoint()) {
            voltage = prevBackspinVoltage;
        }

        voltage = MathUtil.clamp(voltage, 0, 12);

        backspinMotor.setVoltage(voltage);

        prevBackspinVoltage = voltage;

        if (backspinPIDController.atSetpoint()) {
            SmartDashboard.putBoolean("Shooter/BackspinAtTargetRPM", true);
            return Robot.DONE;
        }

        SmartDashboard.putBoolean("Shooter/BackspinAtTargetRPM", false);
        return Robot.CONT;
    }

    /**
     * Sets voltage of the flywheel motor according to a target RPM.
     * @param targetRPM Target RPM.
     * @return Robot.DONE if current RPM is within tolerance, otherwise Robot.CONT.
     */
    public int setFlywheelTargetRPM(double targetRPM) {
        double currentRPM = flywheelMotorEncoder.getVelocity();

        double voltage = flywheelPIDController.calculate(currentRPM, targetRPM);

        flywheelMotor.setVoltage(voltage);

        if (flywheelPIDController.atSetpoint()) {
            SmartDashboard.putBoolean("Shooter/FlywheelAtTargetRPM", true);
            return Robot.DONE;
        }

        SmartDashboard.putBoolean("Shooter/FlywheelAtTargetRPM", false);
        return Robot.CONT;
    }
    
    public void setVelocityWithDelta(double velocity)  {
        double voltage;
        double pidOutput;
    

        if (velocity == prevShooterVelocity){
            voltage = prevShooterVoltage;
        }
        else {
            voltage = velocity/VELOCITY_TO_VOLT_RATIO;
        }
        
        pidOutput = flywheelPIDController.calculate(flywheelMotorEncoder.getVelocity(), velocity);
        voltage = voltage + pidOutput;

        voltage = MathUtil.clamp(voltage, -12.0, 12.0);
        flywheelMotor.setVoltage(voltage);
        if (voltage >= 0.0) {
            backspinMotor.setVoltage(voltage - SHOOTER_MOTOR_DELTA);
        } 
        else {
            backspinMotor.setVoltage(voltage + SHOOTER_MOTOR_DELTA);
        }    
         
        prevShooterVoltage  = voltage;
        prevShooterVelocity = velocity;
        System.out.println("velocity" + flywheelMotorEncoder.getVelocity());
    }

    public void stopMotors() {
        flywheelMotor.stopMotor();
        backspinMotor.stopMotor();
    }

    /******************************************************************************************************
     * 
     * TEST PROGRAMS
     * 
     ******************************************************************************************************/

    public void testVoltageVsVelocity(double voltage) {
        double velocity;

        flywheelMotor.setVoltage(voltage);
        velocity = flywheelMotorEncoder.getVelocity();
        System.out.println("velocity = " + velocity + "    " + voltage);
    
    }
}

/* FOR MOTOR SET FUCNTION
* .1 power = 606 RPM
* .2 power = 1240 RPM
* .3 power = 1866 RPM
* .4 power = 2500 RPM
* .5 power = 3140 RPM
* .6 power = 3783 RPM
* .7 power = 4408 RPM
* .8 power = 5012 RPM
* .9 power = 5600 RPM
*  1 power = 6175 RPM
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