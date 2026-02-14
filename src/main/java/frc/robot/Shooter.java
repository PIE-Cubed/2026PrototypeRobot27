package frc.robot;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * chud shooter class
 */
public class Shooter {

    // private SparkFlex backspinMotor;
    // private SparkBaseConfig backspinMotorConfig;

    private SparkFlex flywheelMotor;
    private SparkBaseConfig flywheelMotorConfig;

    private SparkMax hoodMotor;
    private SparkBaseConfig hoodMotorConfig;

    // private RelativeEncoder backspinMotorEncoder;
    private RelativeEncoder flywheelMotorEncoder;
    private AbsoluteEncoder hoodAbsoluteEncoder;
    private AbsoluteEncoderConfig hoodAbsoluteEncoderConfig;

    // private PIDController backspinPIDController;
    private PIDController flywheelPIDController;
    private PIDController hoodPIDController;
    // private ProfiledPIDController hoodPIDController;
    // private ArmFeedforward hoodFeedForward;

    // Motor IDs
    private final int HOOD_MOTOR_ID = 22;
    // private final int BACKSPIN_MOTOR_ID = 21;
    private final int FLYWHEEL_MOTOR_ID = 20;

    // PID Values
    // private final double BACKSPIN_P = 0.00015;
    // private final double BACKSPIN_I = 0.0;
    // private final double BACKSPIN_D = 0.00003;
    // private final double BACKSPIN_TOLERANCE = 50.0;

    private final double FLYWHEEL_P = 0.0001;
    private final double FLYWHEEL_I = 0.0;
    private final double FLYWHEEL_D = 0.00002;
    private final double FLYWHEEL_TOLERANCE = 50.0;

    // Hood PID and feed forward values determined with ReCalc using estimates of:
    // 50:1 reduction, CoM distance of 4 inches from hinge, and an arm mass of 4 lbs.
    // TODO: Use actual measurements in ReCalc and manually tune values
    private final double HOOD_P = 0.04;
    private final double HOOD_I = 0.0;
    private final double HOOD_D = 0.0;
    private final double HOOD_TOLERANCE = 1.0;

    // private final double HOOD_FFWD_KS = 0.0; // Still need to figure this out
    // private final double HOOD_FFWD_KG = 0.440;
    // private final double HOOD_FFWD_KV = 0.00789;
    // private final double HOOD_FFWD_KA = 0.0000833;

    // private final double HOOD_MAX_VEL_DEG = 1052.63 * 0.85;
    // private final double HOOD_MAX_ACC_DEG = 61919.5 * 0.85;

    // Fully bottomed out is 10 arbitary units.
    private final double HOOD_MIN_ANGLE_DEG = 20;
    private final double HOOD_MAX_ANGLE_DEG = 340;

    private double prevFlywheelVoltage = 0;
    // private double prevBackspinVoltage = 0;

    private static final double VELOCITY_TO_VOLT_RATIO = 540;
    private static final double HOOD_ENCODER_CONVERSION = 360;
    
    public Shooter() {
        // backspinMotor = new SparkFlex(BACKSPIN_MOTOR_ID, MotorType.kBrushless);
        // backspinMotorConfig = new SparkFlexConfig();

        flywheelMotor = new SparkFlex(FLYWHEEL_MOTOR_ID, MotorType.kBrushless);
        flywheelMotorConfig = new SparkFlexConfig();

        hoodMotor = new SparkMax(HOOD_MOTOR_ID, MotorType.kBrushless);
        hoodMotorConfig = new SparkMaxConfig();

        // backspinMotorConfig.idleMode(IdleMode.kCoast);
        // backspinMotorConfig.smartCurrentLimit(Robot.VORTEX_CURRENT_LIMIT);
        // backspinMotorConfig.inverted(true);

        flywheelMotorConfig.idleMode(IdleMode.kCoast);
        flywheelMotorConfig.smartCurrentLimit(Robot.VORTEX_CURRENT_LIMIT);
        flywheelMotorConfig.inverted(true);

        hoodMotorConfig.idleMode(IdleMode.kBrake);
        hoodMotorConfig.smartCurrentLimit(Robot.NEO_550_CURRENT_LIMIT);
        hoodMotorConfig.inverted(true);

        // backspinMotor.configure(backspinMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        flywheelMotor.configure(flywheelMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        hoodMotor.configure(hoodMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        // backspinMotorEncoder = backspinMotor.getEncoder();
        flywheelMotorEncoder = flywheelMotor.getEncoder();
        hoodAbsoluteEncoder = hoodMotor.getAbsoluteEncoder();

        hoodAbsoluteEncoderConfig = new AbsoluteEncoderConfig();
        hoodAbsoluteEncoderConfig.positionConversionFactor(HOOD_ENCODER_CONVERSION);
        hoodAbsoluteEncoderConfig.inverted(true);
        hoodMotorConfig.apply(hoodAbsoluteEncoderConfig);

        // backspinPIDController = new PIDController(BACKSPIN_P, BACKSPIN_I, BACKSPIN_D);
        // backspinPIDController.setTolerance(BACKSPIN_TOLERANCE);

        flywheelPIDController = new PIDController(FLYWHEEL_P, FLYWHEEL_I, FLYWHEEL_D);
        flywheelPIDController.setTolerance(FLYWHEEL_TOLERANCE);
        // hoodFeedForward = new ArmFeedforward(HOOD_FFWD_KS, HOOD_FFWD_KG, HOOD_FFWD_KV, HOOD_FFWD_KA);

        hoodPIDController = new PIDController(HOOD_P, HOOD_I, HOOD_D);
        hoodPIDController.setTolerance(HOOD_TOLERANCE);
        // hoodPIDController = new ProfiledPIDController(
        //     HOOD_P,
        //     HOOD_I,
        //     HOOD_D,
        //     new TrapezoidProfile.Constraints(HOOD_MAX_VEL_DEG, HOOD_MAX_ACC_DEG)
        // );
        // hoodPIDController.setTolerance(HOOD_TOLERANCE);
    }

    public void setMotorRPM(double flywheelRPM, double backspinRPM) {
        // setBackspinMotorVoltage(flywheelRPM / VELOCITY_TO_VOLT_RATIO);
        setFlywheelMotorVoltage(backspinRPM / VELOCITY_TO_VOLT_RATIO);
    }

    /**
     * voltage should be -12 to 12
     * @param flywheelVoltage
     * @param backspinVoltage
     */
    public void setWheelVoltages(double flywheelVoltage, double backspinVoltage) {
        setFlywheelMotorVoltage(flywheelVoltage);
        // setBackspinMotorVoltage(backspinVoltage);
    }

    /**
     * voltage should be -12 to 12
     * @param voltage
     */
    // public void setBackspinMotorVoltage(double voltage) {
    //     backspinMotor.setVoltage(voltage);
    // }

    /**
     * voltage should be -12 to 12
     * @param voltage
     */
    public void setFlywheelMotorVoltage(double voltage) {
        flywheelMotor.setVoltage(voltage);
    }

    /**
     * voltage should be -12 to 12
     * @param voltage
     */
    // public void setHoodMotorVoltage(double voltage) {
    //     hoodMotor.setVoltage(voltage);
    // }

    /*
     * maybe something to do eventually
     */
    public void setTargetArc(double height, double distance) {}

    /**
     * Sets voltage of the shooter motors according to target RPMs.
     * @param targetFlywheelRPM Target RPM of the flywheel motor.
     * @param targetBackspinRPM Target RPM of the backspin motor.
     */
    public void setTargetRPMs(double targetFlywheelRPM) {
        double currentFlywheelRPM = flywheelMotorEncoder.getVelocity();
        // double currentBackspinRPM = backspinMotorEncoder.getVelocity();

        double flywheelVoltage = prevFlywheelVoltage;
        flywheelVoltage = flywheelVoltage + flywheelPIDController.calculate(currentFlywheelRPM, targetFlywheelRPM);

        // double backspinVoltage = prevBackspinVoltage;
        // backspinVoltage = backspinVoltage + backspinPIDController.calculate(currentBackspinRPM, targetBackspinRPM);

        flywheelVoltage = MathUtil.clamp(flywheelVoltage, -12, 12);
        // backspinVoltage = MathUtil.clamp(backspinVoltage, -12, 12);

        flywheelMotor.setVoltage(flywheelVoltage);
        // backspinMotor.setVoltage(backspinVoltage);

        prevFlywheelVoltage = flywheelVoltage;
        // prevBackspinVoltage = backspinVoltage;
    }

    /**
     * Moves the hood according to a target angle. targetAngleDeg should be between HOOD_MIN_ANGLE_DEG and HOOD_MAX_ANGLE_DEG.
     * @param targetAngleDeg The target angle, in degrees. Clamped to HOOD_MIN_ANGLE_DEG and HOOD_MAX_ANGLE_DEG.
     */
    // public void setHoodAngle(double targetAngleDeg) {
    //     targetAngleDeg = MathUtil.clamp(targetAngleDeg, HOOD_MIN_ANGLE_DEG, HOOD_MAX_ANGLE_DEG);

    //     double currPositionDeg = hoodAbsoluteEncoder.getPosition();
    //     double currVelocityDps = hoodAbsoluteEncoder.getVelocity();

    //     double feedback = hoodPIDController.calculate(currPositionDeg, targetAngleDeg);
    //     double feedforward = hoodFeedForward.calculateWithVelocities(
    //         Math.toRadians(currPositionDeg),
    //         Math.toRadians(currVelocityDps),
    //         Math.toRadians(hoodPIDController.getSetpoint().velocity)
    //     );

    //     double voltage = feedback + feedforward;

    //     hoodMotor.setVoltage(voltage);
    // }

    /**
     * Moves the hood according to a target angle. targetAngleDeg should be between HOOD_MIN_ANGLE_DEG and HOOD_MAX_ANGLE_DEG.
     * @param targetAngleDeg The target angle, in degrees. Clamped to HOOD_MIN_ANGLE_DEG and HOOD_MAX_ANGLE_DEG.
     */
    public void setHoodAngle(double targetAngleDeg) {
        targetAngleDeg = MathUtil.clamp(targetAngleDeg, HOOD_MIN_ANGLE_DEG, HOOD_MAX_ANGLE_DEG);

        double currPositionDeg = hoodAbsoluteEncoder.getPosition();
        // double currVelocityDps = hoodAbsoluteEncoder.getVelocity();

        // double feedback = hoodPIDController.calculate(currPositionDeg, targetAngleDeg);
        // double feedforward = hoodFeedForward.calculateWithVelocities(
        //     Math.toRadians(currPositionDeg),
        //     Math.toRadians(currVelocityDps),
        //     Math.toRadians(hoodPIDController.getSetpoint().velocity)
        // );

        // double voltage = feedback + feedforward;

        double voltage = hoodPIDController.calculate(currPositionDeg, targetAngleDeg);

        hoodMotor.setVoltage(voltage);
    }

    public void printWheelRPMs() {
        double currentFlywheelRPM = flywheelMotorEncoder.getVelocity();
        // double currentBackspinRPM = backspinMotorEncoder.getVelocity();

        SmartDashboard.putNumber("Shooter/CurrentFlywheelRPM", currentFlywheelRPM);
        // SmartDashboard.putNumber("Shooter/CurrentBackspinRPM", currentBackspinRPM);
    }

    public void stopWheels() {
        flywheelMotor.stopMotor();
        // backspinMotor.stopMotor();

        // prevBackspinVoltage = 0;
        prevFlywheelVoltage = 0;
    }

    public void stopHood() {
        hoodMotor.stopMotor();
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
