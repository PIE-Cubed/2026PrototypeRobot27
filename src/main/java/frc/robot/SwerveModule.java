package frc.robot;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

public class SwerveModule {
    private SparkFlex       driveMotor;
    private SparkBaseConfig driveMotorConfig;

    private SparkMax        rotateMotor;
    private SparkBaseConfig rotateMotorConfig;

    // Setting constant variables
    public final int DRIVE_MOTOR_CURRENT_LIMIT = 65;
    private final int ROTATE_MOTOR_CURRENT_LIMIT = 40;

    // Encoder creation
    private RelativeEncoder driveEncoder;
    private EncoderConfig driveEncoderConfig;

    private SparkAbsoluteEncoder absoluteEncoder;
    private AbsoluteEncoderConfig absoluteEncoderConfig;

    private PIDController rotatePIDController;

    private boolean invertedDriveMotor;

    // PID
    // TODO: Calibrate PIDs
    private final double ROTATE_P = 0.007;  //.00384
    private final double ROTATE_I = 0;
    private final double ROTATE_D = 0;
    private final double ROTATE_PID_TOLERANCE = 3;  // In degrees
    private final double ROTATE_PID_TOLERANCE_LOOSE = 6;
    
    // Drive motor conversion factor cacluations
    private final double WHEEL_DIAMETER_INCHES = 2.970; // 2.88
    private final double WHEEL_ROTATION_FEET = Math.PI * WHEEL_DIAMETER_INCHES / 12;   // Feet per rotation (circumference)
    private final double ROTATIONS_PER_TICK  = 1 / 5.08 / 1;    // 1 / external gearing / gearbox
    // Drive Motor Conversion Factors
    private final double DRIVE_POS_CONVERSION_FACTOR = WHEEL_ROTATION_FEET * ROTATIONS_PER_TICK; // Feet per tick
    private final double DRIVE_VEL_CONVERSION_FACTOR = DRIVE_POS_CONVERSION_FACTOR / 60;         // Feet per second

    // Rotate Motor Conversion Factor
    private final double ROTATE_ENCODER_CONVERSION = 360; // Convert the rotate motor's encoder to degrees

    public SwerveModule(int driveID, int rotateID, boolean invertDriveMotor) {
        this.invertedDriveMotor = invertDriveMotor;

        // Use these for the 2025
        driveMotor       = new SparkFlex(driveID, MotorType.kBrushless);
        driveMotorConfig = new SparkFlexConfig();

        // Drive motor init and config
       // driveMotor = new SparkMax(driveID, MotorType.kBrushless);
       // driveMotorConfig = new SparkMaxConfig();
        driveMotorConfig.smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);
        driveMotorConfig.idleMode(IdleMode.kBrake);
        driveMotorConfig.inverted(this.invertedDriveMotor);
        
        // Rotate motor init and config
        rotateMotor       = new SparkMax(rotateID, MotorType.kBrushless);
        rotateMotorConfig = new SparkMaxConfig();
        rotateMotorConfig.smartCurrentLimit(ROTATE_MOTOR_CURRENT_LIMIT);
        rotateMotorConfig.idleMode(IdleMode.kBrake);

        // Drive encoder configuration/setup
        driveEncoder = driveMotor.getEncoder();
        driveEncoder.setPosition(0.0);

        driveEncoderConfig = new EncoderConfig();
        driveEncoderConfig.positionConversionFactor(DRIVE_POS_CONVERSION_FACTOR);
        driveEncoderConfig.velocityConversionFactor(DRIVE_VEL_CONVERSION_FACTOR);
        driveMotorConfig.apply(driveEncoderConfig); // Apply the encoder configuration to the SparkMax configuration

        // Rotate encoder configuration/setup
        absoluteEncoder = rotateMotor.getAbsoluteEncoder();
        
        absoluteEncoderConfig = new AbsoluteEncoderConfig();
        absoluteEncoderConfig.positionConversionFactor(ROTATE_ENCODER_CONVERSION);
        absoluteEncoderConfig.inverted(true);
        rotateMotorConfig.apply(absoluteEncoderConfig); // Apply the encoder configuration to the SparkMax configuration

        // Rotate PID controller configuration/setup
        rotatePIDController = new PIDController(ROTATE_P, ROTATE_I, ROTATE_D);
        rotatePIDController.enableContinuousInput(0, 360);
        rotatePIDController.setTolerance(ROTATE_PID_TOLERANCE);
        rotatePIDController.setIntegratorRange(-0.1, 0.1);
        rotatePIDController.reset();



        // Apply the configurations(motor and encoder) to the SparkMax 
        rotateMotor.configure(rotateMotorConfig, 
                              ResetMode.kNoResetSafeParameters, 
                              PersistMode.kPersistParameters);

        driveMotor.configure(driveMotorConfig, 
                             ResetMode.kNoResetSafeParameters, 
                             PersistMode.kPersistParameters);
    }

    public void setDesiredState(SwerveModuleState swerveModuleState, boolean optimize) { 
        // Optimizes the wheel movements
        /* When you optimize the swerve module state you minimize the wheel rotation.
         * For instance instead of rotating 180 degrees and driving forward you can 
         * just drive in reverse and not rotate.  The optimized state will figure this 
         * out for you.
         */
        if(optimize) {
            swerveModuleState.optimize(new Rotation2d( MathUtil.angleModulus(Units.degreesToRadians(absoluteEncoder.getPosition()))));
        }

        double currentAngleDegrees = absoluteEncoder.getPosition();
        double targetAngleDegrees  = swerveModuleState.angle.getDegrees();
        double rotatePower         = rotatePIDController.calculate(currentAngleDegrees, targetAngleDegrees);

        driveMotor.set(MathUtil.clamp(swerveModuleState.speedMetersPerSecond, -1.0, 1.0));
        rotateMotor.set(MathUtil.clamp(rotatePower, -1.0, 1.0));
    }

    public void setDriveMotorPower(double power) {
        driveMotor.set(MathUtil.clamp(power, -1, 1));
    }

    public void setRotateMotorPower(double power) {
        rotateMotor.set(MathUtil.clamp(power, -1, 1));
    }

    /// Returns SwerveModulePosition, with `distanceMeters` in feet
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(
            driveEncoder.getPosition(),
            new Rotation2d(
                MathUtil.angleModulus(
                    Units.degreesToRadians(absoluteEncoder.getPosition())
                )
            )
        );
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(
            driveEncoder.getVelocity(),
            new Rotation2d(
                MathUtil.angleModulus(
                    Units.degreesToRadians(absoluteEncoder.getPosition())
                )
            )
        );
    }

    public void setLooseTolerance() {
        rotatePIDController.setTolerance(ROTATE_PID_TOLERANCE_LOOSE);
    }

    public void setTightTolerance() {
        rotatePIDController.setTolerance(ROTATE_PID_TOLERANCE);
    }
    
    public REVLibError resetPositionEncoder() {
        return driveEncoder.setPosition(0.0);
    }

    /**
     * <p>Set the drive motor smart current limit
     * <p>Performs no checks on currentLimit
     * @param currentLimit
     * @return REVLibError kOk on success
     */
    public REVLibError setDriveCurrentLimit(int currentLimit) {
        driveMotorConfig.smartCurrentLimit(currentLimit);

        return driveMotor.configure(driveMotorConfig, 
                             ResetMode.kNoResetSafeParameters, 
                             PersistMode.kPersistParameters);
    }

    /**
     * Checks if the rotate controller is at the setpoint.
     * @return atSetpoint
     */
    public boolean rotateControllerAtSetpoint() {
        return rotatePIDController.atSetpoint();
    }
}