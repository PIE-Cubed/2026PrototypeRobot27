package frc.robot;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import java.util.List;
import org.photonvision.EstimatedRobotPose;

public class Drive {

    public static final double SWERVE_DIST_FROM_CENTER = 0.29845;
    public static final Translation2d centerLocation = new Translation2d(0, 0);
    public static final Translation2d frontLeftLocation = new Translation2d(
        SWERVE_DIST_FROM_CENTER,
        SWERVE_DIST_FROM_CENTER
    );
    public static final Translation2d frontRightLocation = new Translation2d(
        SWERVE_DIST_FROM_CENTER,
        -SWERVE_DIST_FROM_CENTER
    );
    public static final Translation2d backLeftLocation = new Translation2d(
        -SWERVE_DIST_FROM_CENTER,
        SWERVE_DIST_FROM_CENTER
    );
    public static final Translation2d backRightLocation = new Translation2d(
        -SWERVE_DIST_FROM_CENTER,
        -SWERVE_DIST_FROM_CENTER
    );

    public SwerveDriveKinematics swerveDriveKinematics;

    private SwerveModule frontLeft;
    private SwerveModule frontRight;
    private SwerveModule backLeft;
    private SwerveModule backRight;

    // on-the-fly drive variables
    private final double ROBOT_MASS_KG = 37; // robot mass in kilograms (for pathPlanner)
    private final double ROBOT_MOI = 6.31; // robot moment of inertia, from the cad so approximate (for pathPlanner)
    private PathPlannerPath otfPath; // OTF means on-the-fly
    private PathPlannerTrajectory otfTrajectory;
    private boolean otfFirstTime = true;
    private Timer otfTimer;

    // AprilTag PID controllers
    private PIDController otfStrafePID;
    private PIDController otfForwardPID;
    private PIDController otfRotatePID;
    private PIDController choreoRotatePID;

    // OTF PID values
    private final double OTF_S_P = 3;
    private final double OTF_S_I = 0;
    private final double OTF_S_D = 0;
    private final double OTF_S_I_RANGE = 0;
    private final double OTF_S_I_ZONE = 0;

    private final double OTF_F_P = 3;
    private final double OTF_F_I = 0;
    private final double OTF_F_D = 0;
    private final double OTF_F_I_RANGE = 0;
    private final double OTF_F_I_ZONE = 0;

    private final double OTF_R_P = 0.1;
    private final double OTF_R_I = 0;
    private final double OTF_R_D = 0;
    private final double OTF_R_I_RANGE = 0;
    private final double OTF_R_I_ZONE = 0;

    private final double CHOREO_R_P = 0.8;
    private final double CHOREO_R_I = 0;
    private final double CHOREO_R_D = 0;
    private final double CHOREO_R_I_RANGE = 0;
    private final double CHOREO_R_I_ZONE = 0;

    private final double OTF_SF_TOLERANCE = 0;
    private final double OTF_R_TOLERANCE = 0;
    private final double CHOREO_R_TOLERANCE = 0;

    private PIDController rotatePID;
    private final double ROTATE_P = 0.009;
    private final double ROTATE_I = 0.0;
    private final double ROTATE_D = 0.0;

    private final double MAX_WHEEL_POWER = 1;

    // Standard deviations (trust values) for encoders and April Tags
    // The lower the numbers the more trustworthy the prediction from that source is
    private final Vector<N3> ENCODER_STD_DEV = VecBuilder.fill(0.1, 0.1, 0.1);
    private final Vector<N3> APRILTAG_STD_DEV = VecBuilder.fill(0.3, 0.3, 0.5);

    private static SwerveDrivePoseEstimator aprilTagsEstimator;

    private Pose2d lastPose = new Pose2d();
    private Pose2d currPose = new Pose2d();

    private Timer timer;

    private AHRS ahrs;

    public Drive() {
        try {
            ahrs = new AHRS(NavXComType.kMXP_SPI);
        } catch (RuntimeException ex) {
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

        frontLeft = new SwerveModule(14, 15, true);
        frontRight = new SwerveModule(16, 17, false);
        backLeft = new SwerveModule(12, 13, true);
        backRight = new SwerveModule(10, 11, false);

        rotatePID = new PIDController(ROTATE_P, ROTATE_I, ROTATE_D);

        // Strafe PID for OTF drive
        otfStrafePID = new PIDController(OTF_S_P, OTF_S_I, OTF_S_D);
        otfStrafePID.setTolerance(OTF_SF_TOLERANCE);
        otfStrafePID.setIntegratorRange(-OTF_S_I_RANGE, OTF_S_I_RANGE);
        otfStrafePID.setIZone(OTF_S_I_ZONE);

        // Forward PID for OTF drive
        otfForwardPID = new PIDController(OTF_F_P, OTF_F_I, OTF_F_D);
        otfForwardPID.setTolerance(OTF_SF_TOLERANCE);
        otfForwardPID.setIntegratorRange(-OTF_F_I_RANGE, OTF_F_I_RANGE);
        otfForwardPID.setIZone(OTF_F_I_ZONE);

        // Rotate PID for OTF drive
        otfRotatePID = new PIDController(OTF_R_P, OTF_R_I, OTF_R_D);
        otfRotatePID.setTolerance(OTF_R_TOLERANCE);
        otfRotatePID.setIntegratorRange(-OTF_R_I_RANGE, OTF_R_I_RANGE);
        otfRotatePID.setIZone(OTF_R_I_ZONE);

        choreoRotatePID = new PIDController(CHOREO_R_P, CHOREO_R_I, CHOREO_R_D);
        choreoRotatePID.setTolerance(CHOREO_R_TOLERANCE);
        choreoRotatePID.setIntegratorRange(-CHOREO_R_I_RANGE, CHOREO_R_I_RANGE);
        choreoRotatePID.setIZone(CHOREO_R_I_ZONE);

        timer = new Timer();

        SwerveModulePosition[] initialPosition = getModulePositions();
        Rotation2d initialRotation = new Rotation2d(getYawRadians());

        // encoderEstimator = new SwerveDriveOdometry(
        //     swerveDriveKinematics,
        //     initialRotation,
        //     initialPosition,
        //     new Pose2d(0, 0, new Rotation2d(0))
        // );

        aprilTagsEstimator = new SwerveDrivePoseEstimator(
            swerveDriveKinematics,
            initialRotation,
            initialPosition,
            new Pose2d(0, 0, new Rotation2d(0)),
            ENCODER_STD_DEV,
            APRILTAG_STD_DEV
        );

        timer.restart();
    }

    public void teleopDrive(
        double forwardPowerFwdPos,
        double strafePowerLeftPos,
        double rotatePowerCcwPos,
        boolean fieldDrive
    ) {
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
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        forwardPowerFwdPos,
                        strafePowerLeftPos,
                        rotatePowerCcwPos,
                        new Rotation2d(getYawRadians())
                    )
                    : new ChassisSpeeds(
                        forwardPowerFwdPos,
                        strafePowerLeftPos,
                        rotatePowerCcwPos
                    )
            );

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_WHEEL_POWER);

        setModuleStates(swerveModuleStates, true);
    }

    /**
     * <p> Drive in teleop, either relative to the robot or the field
     * <p> Allows a center of rotation different from the center of the robot
     * @param forwardPower Positive goes forward
     * @param strafePower Positive goes right
     * @param rotatePower Positive is counter clockwise (might be clockwise)
     * @param fieldDrive Whether to drive with field relative speeds
     * @param centerOfRotation The offset for the center of rotation (in meters)
     */
    public void teleopDrive(
        double forwardPower,
        double strafePower,
        double rotatePower,
        boolean fieldDrive,
        Translation2d centerOfRotation
    ) {
        /*
         * FieldRelativeSpeeds:
         *  Positive for away from alliance wall
         *  Positive for left from alliance wall
         *  Positive for counter clockwise
         *  0 for facing down the field, counter clockwise positive
         * ChassisSpeeds:
         *  Positive for forward (forwardPower)
         *  Positive for left (strafePower)
         *  Positive for counter clockwise (rotatePower) (might be clockwise)
         */

        // System.out.println("fwd" + forwardPower + " ---- strf" + strafePower*-1 + " ---- rot:" + rotatePower);

        SwerveModuleState[] swerveModuleStates =
            swerveDriveKinematics.toSwerveModuleStates(
                fieldDrive
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        forwardPower,
                        strafePower,
                        rotatePower,
                        new Rotation2d(getYawRadians())
                    )
                    : new ChassisSpeeds(forwardPower, strafePower, rotatePower),
                centerOfRotation
            );

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_WHEEL_POWER);

        // The SwerveModuleStates array index used must match the order from the SwerveDriveKinematics instantiation
        setModuleStates(swerveModuleStates, true);
    }

    /**
     * Drives using velocities instead of a -1 to 1 value, either relative to the robot or the field. Check SwerveModule.java for max velocity.
     * (this actually uses the odometry instead of the AHRS so it is consistently facing one direction)
     * @param forwardMPS Positive goes forward or towards the opposite alliance wall
     * @param strafeMPS Positive goes right or towards your alliance's right wall
     * @param rotateDPS Positive is counter clockwise (might be clockwise actually)
     * @param fieldDrive Whether to drive with field relative speeds
     * @param isRedAlliance Whether the robot is on the red alliance or not. Set to false if using for autonomous. (if true, changes robot rotation by 180 degrees so it is facing the other way)
     */
    public void velocityDrive(
        double forwardMPS,
        double strafeMPS,
        double rotateDPS,
        boolean fieldDrive,
        boolean isRedAlliance
    ) {
        /*
         * FieldRelativeSpeeds:
         *  Positive for away from your alliance wall
         *  Positive for away from your alliance's right wall
         *  Positive for counter clockwise
         *  0 for facing down the field, counter clockwise positive
         * ChassisSpeeds:
         *  Positive for forward, in meters per second (strafeMPS)
         *  Positive for left, in meters per second (strafeMPS)
         *  Positive for counter clockwise rotation in degrees per second (rotateDPS)
         */
        ChassisSpeeds robotSpeeds = new ChassisSpeeds(
            forwardMPS / SwerveModule.MAX_DRIVE_VEL_MPS,
            strafeMPS / SwerveModule.MAX_DRIVE_VEL_MPS,
            rotateDPS / Units.degreesToRadians(SwerveModule.MAX_ROTATE_VEL_DPS)
        );

        Rotation2d fieldRotation;

        if (isRedAlliance) {
            fieldRotation = getPose().getRotation().rotateBy(new Rotation2d(180));
        } else {
            fieldRotation = getPose().getRotation();
        }

        SwerveModuleState[] swerveModuleStates =
            swerveDriveKinematics.toSwerveModuleStates(
                fieldDrive
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(robotSpeeds, fieldRotation)
                    : robotSpeeds
            );

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_WHEEL_POWER);

        // Check for deadzone
        setModuleStates(swerveModuleStates, true);
    }

    /**
     * <p> Drives the robot to a target pose on the field.
     * <p> The PID controllers are tuned for PathPlanner trajectory samples,
     *     not for large movements which driveToAprilTag's PIDs are tuned for.
     * <p> OTF means on-the-fly
     * @param targetPose
     * The target robot pose to drive to based on field coordinates (x = 0, y = 0 at the blue alliance's right corner from blue driver perspective)
     * TODO apparently pathplanner already gives you chassisSpeeds so you don't need PIDs, TEST THIS IMMEDIATELY
     */
    public int otfDriveTo(Pose2d targetPose) {
        Pose2d pose = getPose();

        if (
            otfForwardPID.atSetpoint() &&
            otfStrafePID.atSetpoint() &&
            otfRotatePID.atSetpoint() &&
            otfTimer.get() >= otfTrajectory.getTotalTimeSeconds()
        ) {
            System.out.println("OTF drive finished");

            otfFirstTime = true;

            stopWheels();

            otfTimer.stop();
            otfTimer.reset();

            return Robot.DONE;
        }

        if (otfFirstTime == true) {
            Pose2d initialPose = getPose();

            //System.out.println("yaw rate: " + getYawRate());
            //System.out.println("initial pose: " + initialPose);

            List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                initialPose,
                // initialPose.interpolate(targetPose, 0.5),
                targetPose
            );

            // for (Waypoint point : waypoints) {
            //     System.out.println("waypoint anchor: " + point.anchor());
            // }

            // used to be unlimited constraints HOWEVER PathPlanner apparently sets all the max values EXTREMELY low for safety
            PathConstraints constraints = new PathConstraints(
                2,
                2.5,
                Units.degreesToRadians(270),
                Units.degreesToRadians(270),
                12,
                false
            );

            otfPath = new PathPlannerPath(
                waypoints,
                constraints,
                new IdealStartingState(
                    0/* change this to current velocity later */,
                    initialPose.getRotation()
                ),
                new GoalEndState(0, targetPose.getRotation())
            );

            // for (PathPoint point : otfPath.getAllPathPoints()) {
            //     System.out.println("OTF path sample" + point.position);
            // }

            ChassisSpeeds initialSpeeds = new ChassisSpeeds(0, 0, getYawRateRadians());

            otfTrajectory = new PathPlannerTrajectory(
                otfPath,
                initialSpeeds,
                initialPose.getRotation(),
                new RobotConfig(
                    ROBOT_MASS_KG,
                    ROBOT_MOI,
                    SwerveModule.swerveModuleConfig,
                    frontLeftLocation,
                    frontRightLocation,
                    backLeftLocation,
                    backRightLocation
                )
            );

            // for (PathPlannerTrajectoryState point : otfTrajectory.getStates()) {
            //     System.out.println("time: " + point.timeSeconds + " pose: " + point.pose + " field-relative speeds: " + point.fieldSpeeds);
            // }

            // System.out.println("Trajectory time: " + otfTrajectory.getTotalTimeSeconds());
            // System.out.println("Trajectory states: " + otfTrajectory.getStates().size());

            otfFirstTime = false;

            otfTimer.stop();
            otfTimer.reset();
            otfTimer.start();
        }

        PathPlannerTrajectoryState sampleState = otfTrajectory.sample(otfTimer.get());
        Pose2d samplePose = sampleState.pose;
        ChassisSpeeds sampleSpeeds = sampleState.fieldSpeeds;

        velocityDrive(
            sampleSpeeds.vxMetersPerSecond +
            otfForwardPID.calculate(pose.getX(), samplePose.getX()),
            sampleSpeeds.vyMetersPerSecond +
            otfStrafePID.calculate(pose.getY(), samplePose.getY()),
            sampleSpeeds.omegaRadiansPerSecond +
            otfRotatePID.calculate(
                pose.getRotation().getDegrees(),
                samplePose.getRotation().getDegrees()
            ),
            true,
            false
        );

        //DogLog.log("Drive/otfSamplePose", samplePose);
        /*
        ChassisSpeeds speeds = 
            ChassisSpeeds.fromFieldRelativeSpeeds(otfForwardPID.calculate(pose.getX(),                     samplePose.getX()),
                                                  otfStrafePID.calculate( pose.getY(),                     samplePose.getY()),
                                                  otfRotatePID.calculate( pose.getRotation().getDegrees(), samplePose.getRotation().getDegrees()),
                                                  pose.getRotation());

        SwerveModuleState[] swerveModuleStates = swerveDriveKinematics.toSwerveModuleStates(speeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_WHEEL_SPEED);

        setModuleStates(swerveModuleStates, true);
        */

        return Robot.CONT;
    }

    public void otfReset() {
        otfFirstTime = true;
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
        frontLeft.setDesiredState(
            new SwerveModuleState(0, new Rotation2d(Math.PI / 4)),
            false
        );
        frontRight.setDesiredState(
            new SwerveModuleState(0, new Rotation2d(-Math.PI / 4)),
            false
        );
        backLeft.setDesiredState(
            new SwerveModuleState(0, new Rotation2d(-Math.PI / 4)),
            false
        );
        backRight.setDesiredState(
            new SwerveModuleState(0, new Rotation2d(Math.PI / 4)),
            false
        );
    }

    /**
     * Sets module state
     * @param moduleStates Input module states
     * @param optimize Whether to optimize the state or use absolute positions
     */
    public void setModuleStates(SwerveModuleState[] moduleStates, boolean optimize) {
        frontLeft.setDesiredState(moduleStates[0], optimize);
        frontRight.setDesiredState(moduleStates[1], optimize);
        backLeft.setDesiredState(moduleStates[2], optimize);
        backRight.setDesiredState(moduleStates[3], optimize);
    }

    /**
     * @return The robot yaw rate, measured in degrees per second.
     */
    public double getYawRateDegrees() {
        return ahrs.getRate();
    }

    /**
     * @return The robot yaw rate, measured in radians per second.
     */
    public double getYawRateRadians() {
        return Math.toRadians(ahrs.getRate());
    }

    /**
     * @return A SwerveModulePosition[] containing each module's current position.
     */
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeft.getModulePosition(),
            frontRight.getModulePosition(),
            backLeft.getModulePosition(),
            backRight.getModulePosition(),
        };
    }

    public Translation2d getCenterOfRotation(
        double rotatePowerCcwPos,
        double rightStickY
    ) {
        Translation2d centerOfRotation = centerLocation;

        if (rightStickY >= 0.2) {
            if (rotatePowerCcwPos < 0) {
                centerOfRotation = frontRightLocation;
            } else if (rotatePowerCcwPos > 0) {
                centerOfRotation = Drive.frontLeftLocation;
            } else {
                centerOfRotation = Drive.centerLocation;
            }
        } else if (rightStickY <= -0.2) {
            if (rotatePowerCcwPos < 0) {
                centerOfRotation = Drive.backRightLocation;
            } else if (rotatePowerCcwPos > 0) {
                centerOfRotation = Drive.backLeftLocation;
            } else {
                centerOfRotation = Drive.centerLocation;
            }
        } else {
            centerOfRotation = Drive.centerLocation;
        }

        return centerOfRotation;
    }

    public void updatePoseEstimator() {
        SwerveModulePosition[] currentPosition = getModulePositions();
        Rotation2d currentRotation = new Rotation2d(getYawRadians());

        // Update vision estimator with encoder data
        aprilTagsEstimator.updateWithTime(timer.get(), currentRotation, currentPosition);

        lastPose = currPose;
        currPose = getPose();
    }

    /**
     * Adds a vision pose estimate to the pose estimator.
     * Does not need to be called every loop as long as updatePoseEstimator is called.
     * @param visionEst The estimated pose from the cameras.
     */
    public void addVisionMeasurement(
        EstimatedRobotPose visionEst,
        Matrix<N3, N3> stdDevs
    ) {
        aprilTagsEstimator.addVisionMeasurement(
            visionEst.estimatedPose.toPose2d(),
            visionEst.timestampSeconds,
            stdDevs.extractColumnVector(0) // Standard deviations of the camera
        );
    }

    /**
     * </p> Resets the estimator to a given pose.
     * </p> Gyro angle and swerve module positions don't have to be reset beforehand
     *      as the estimators automatically creates offsets.
     *
     * @param newPose The Pose2d to reset to.
     */
    public void reset(Pose2d newPose) {
        SwerveModulePosition[] currentPosition = getModulePositions();
        Rotation2d gyro = new Rotation2d(getYawRadians());

        aprilTagsEstimator.resetPosition(gyro, currentPosition, newPose);
    }

    /**
     * </p> Gets the current AprilTag-assisted field position.
     *      If there is no vision estimate it relies on encoder pose.
     * </p> Refer to the WPILib docs for specifics on field-based odometry.
     *
     * @return The estimated Pose. (in meters)
     */
    public static Pose2d getPose() {
        return aprilTagsEstimator.getEstimatedPosition();
    }

    /**
     * Returns the current velocity of the robot.
     */
    public Transform2d getVelocity() {
        // Multiplied by the ammount of loops per second to get meters per second
        Transform2d velocityMeters = currPose.minus(lastPose).times(50);

        return velocityMeters;
    }
}
