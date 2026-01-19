package frc.robot;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

public class Drive {
    public static final double SWERVE_DIST_FROM_CENTER = 0.3254375;
    public static final Translation2d centerLocation = new Translation2d(0, 0);
    public static final Translation2d frontLeftLocation = new Translation2d(SWERVE_DIST_FROM_CENTER, SWERVE_DIST_FROM_CENTER);
    public static final Translation2d frontRightLocation = new Translation2d(SWERVE_DIST_FROM_CENTER, -SWERVE_DIST_FROM_CENTER);
    public static final Translation2d backLeftLocation = new Translation2d(-SWERVE_DIST_FROM_CENTER, SWERVE_DIST_FROM_CENTER);
    public static final Translation2d backRightLocation = new Translation2d(-SWERVE_DIST_FROM_CENTER, -SWERVE_DIST_FROM_CENTER);

    public SwerveDriveKinematics swerveDriveKinematics;

    private SwerveModule frontLeft;
    private SwerveModule frontRight;
    private SwerveModule backLeft;
    private SwerveModule backRight;

    private PIDController rotatePID;
    private final double ROTATE_P = 0.009;
    private final double ROTATE_I = 0.0;
    private final double ROTATE_D = 0.0;

    private final double MAX_WHEEL_POWER = 1;

    private AHRS ahrs;

    public Drive() {
        try {
            ahrs = new AHRS(NavXComType.kMXP_SPI);
        }
        catch (RuntimeException ex) {
            System.out.println("Failed to instanciate navX");
        }

        ahrs.reset();

        while (ahrs.isConnected() == false) {
            System.out.print(".");
        }

        while (ahrs.isCalibrating() == true) {
            System.out.print(".");
        }

        ahrs.zeroYaw();

        swerveDriveKinematics = new SwerveDriveKinematics(
            frontLeftLocation,
            frontRightLocation,
            backLeftLocation,
            backRightLocation
        );

        frontLeft  = new SwerveModule(14, 15, true);
        frontRight = new SwerveModule(16, 17, false);
        backLeft   = new SwerveModule(12, 13, true);
        backRight  = new SwerveModule(10, 11, false);

        rotatePID = new PIDController(ROTATE_P, ROTATE_I, ROTATE_D);
    }

    public void teleopDrive(double forwardPowerFwdPos, double strafePowerLeftPos, double rotatePowerCcwPos, boolean fieldDrive) {
        /*
         * FieldRelativeSpeeds:
         *  Positive for away from alliance wall
         *  Positive for left from alliance wall
         *  Positive for counter clockwise
         *  0 for facing down the field, counter clockwise positive
         * ChassisSpeeds:
         *  Positive for forward (forwardPower)
         *  Positive for left (strafePower)
         *  Positive for counter clockwise (rotatePower)
         */
        SwerveModuleState[] swerveModuleStates =
            swerveDriveKinematics.toSwerveModuleStates(
                fieldDrive 
                ? ChassisSpeeds.fromFieldRelativeSpeeds(forwardPowerFwdPos, strafePowerLeftPos, rotatePowerCcwPos, new Rotation2d( getYawRadians() )) 
                : new ChassisSpeeds(forwardPowerFwdPos, strafePowerLeftPos, rotatePowerCcwPos)
            );

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_WHEEL_POWER);

        frontLeft.setDesiredState(swerveModuleStates[0], true);
        frontRight.setDesiredState(swerveModuleStates[1], true);
        backLeft.setDesiredState(swerveModuleStates[2], true);
        backRight.setDesiredState(swerveModuleStates[3], true);
    }

    public double getYawRadians() {
        return MathUtil.angleModulus(-Units.degreesToRadians(ahrs.getYaw()));
    }

    public void stopWheels() {
        frontLeft.setDriveMotorPower(0.0);
        frontLeft.setRotateMotorPower(0.0);

        frontRight.setDriveMotorPower(0.0);
        frontRight.setRotateMotorPower(0.0);
        
        backLeft.setDriveMotorPower(0.0);
        backLeft.setRotateMotorPower(0.0);
        
        backRight.setDriveMotorPower(0.0);
        backRight.setRotateMotorPower(0.0);
    }

    /**
    * Zero the gyro
    */
    public void resetGyro() {
        ahrs.zeroYaw();
    }

    /**
    * Crosses the wheels and makes the robot impossible to move.
    */
    public void lockWheels() {
        frontLeft.setDesiredState(new SwerveModuleState(0, new Rotation2d( Math.PI / 4 )), false);
        frontRight.setDesiredState(new SwerveModuleState(0, new Rotation2d( -Math.PI / 4 )), false);
        backLeft.setDesiredState(new SwerveModuleState(0, new Rotation2d( -Math.PI / 4 )), false);
        backRight.setDesiredState(new SwerveModuleState(0, new Rotation2d( Math.PI / 4 )), false);       
    }
}
