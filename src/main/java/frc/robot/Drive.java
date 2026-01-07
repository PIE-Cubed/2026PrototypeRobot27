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

    private final double MAX_WHEEL_SPEED = 1;

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

        // TODO: get real ids
        frontLeft  = new SwerveModule(14, 15, false);
        frontRight = new SwerveModule(16, 17, true);
        backLeft   = new SwerveModule(12, 13, false);
        backRight  = new SwerveModule(10, 11, true);

        rotatePID = new PIDController(ROTATE_P, ROTATE_I, ROTATE_D);
    }

    public void teleopDrive(double forwardPower, double strafePower, double rotatePower, boolean fieldDrive) {
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
                ? ChassisSpeeds.fromFieldRelativeSpeeds(forwardPower, strafePower*-1, rotatePower, new Rotation2d( getYawRadians() )) 
                : new ChassisSpeeds(forwardPower, strafePower * -1, rotatePower)
            );

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_WHEEL_SPEED);

        frontLeft.setDesiredState(swerveModuleStates[0], false);
        frontRight.setDesiredState(swerveModuleStates[1], false);
        backLeft.setDesiredState(swerveModuleStates[2], false);
        backRight.setDesiredState(swerveModuleStates[3], false);

    }

    public double getYawRadians() {
        return MathUtil.angleModulus(-Units.degreesToRadians(ahrs.getYaw()));
    }
}
