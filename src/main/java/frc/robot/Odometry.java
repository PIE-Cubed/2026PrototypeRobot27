package frc.robot;

import java.util.ArrayList;
import java.util.List;

import org.ejml.simple.SimpleMatrix;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

// import dev.doglog.DogLog;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.util.Logger;

/** Add your docs here. */
public class Odometry {
    public static int algaeHeight = 2;
    public static List<PhotonPipelineResult> cameraResults;
    public static PhotonPipelineResult       latestResult;

    private final int REQUIRED_APRILTAGS = 2; // Number of required AprilTags to update the AprilTag estimator
    private final double MAX_YAW_RATE_DEGREES = 360; // Maximum angular velocity(degrees/s) to update AprilTag estimator

    // Standard deviations(trust values) for encoders and April Tags
    // The lower the numbers the more trustworthy the prediction from that source is
    private final Vector<N3> ENCODER_STD_DEV  = VecBuilder.fill(0.1, 0.1, 0.1);
    private final Vector<N3> APRILTAG_STD_DEV = VecBuilder.fill(0.3, 0.3, 0.5);

    // Distances from bottom center of robot to each camera
    // When rotation is 0 for all axes the Y axis is parallel .
    private final Transform3d ROBOT_TO_CAMERA1 = new Transform3d(Units.inchesToMeters(14.532183), Units.inchesToMeters(0), Units.inchesToMeters(6.081022),
                                                 new Rotation3d( Units.degreesToRadians(0),     Units.degreesToRadians(-23),    Units.degreesToRadians(0)));
    // private final Transform3d ROBOT_TO_CAMERA2 = new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0));
    // private final Transform3d ROBOT_TO_CAMERA3 = new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0));
    // private final Transform3d ROBOT_TO_CAMERA4 = new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0));

    // Estimators
    private SwerveDriveOdometry encoderEstimator;        // Might not need this as aprilTagsEstimator also uses the encoders
    private SwerveDrivePoseEstimator aprilTagsEstimator;
    private PhotonPoseEstimator camera1PoseEstimator; // Photon Vision estimators
    // private PhotonPoseEstimator camera2PoseEstimator; //
    // private PhotonPoseEstimator camera3PoseEstimator; //
    // private PhotonPoseEstimator camera4PoseEstimator; //

    // Photon Vision cameras
    private PhotonCamera camera1;
    private List<PhotonPipelineResult> camera1Results;
    // private PhotonCamera camera2;
    // private List<PhotonPipelineResult> camera2Results;
    // private PhotonCamera camera3;
    // private List<PhotonPipelineResult> camera3Results;
    // private PhotonCamera camera4;
    // private List<PhotonPipelineResult> camera4Results;

    private EstimatedRobotPose camera1Pose3d;
    // private EstimatedRobotPose camera2Pose3d;
    // private EstimatedRobotPose camera3Pose3d;
    // private EstimatedRobotPose camera4Pose3d;
    
    public Drive drive;

    private int noResultsCount = 0;
    private int c1noResultsCount = 0;
    // private int c2noResultsCount = 0;
    // private int c3noResultsCount = 0;
    // private int c4noResultsCount = 0;

    private Pose2d lastPose = new Pose2d();
    private Pose2d currPose = new Pose2d();

    public Odometry(Drive drive) {
        this.drive = drive;

        // Instantiate the PhotonCameras
        camera1 = new PhotonCamera("camera1");
        // camera2 = new PhotonCamera("camera2");
        // camera3 = new PhotonCamera("camera3");
        // camera4 = new PhotonCamera("camera4");

        SwerveModulePosition[] initialPosition =  drive.getModulePositions();
        Rotation2d initialRotation = new Rotation2d(drive.getYawRadians());

        encoderEstimator = new SwerveDriveOdometry(
            drive.swerveDriveKinematics, 
            initialRotation, 
            initialPosition, 
            new Pose2d(0, 0, new Rotation2d(0))
        );

        aprilTagsEstimator = new SwerveDrivePoseEstimator(
            drive.swerveDriveKinematics, 
            initialRotation, 
            initialPosition, 
            new Pose2d(0, 0, new Rotation2d(0)), 
            ENCODER_STD_DEV, 
            APRILTAG_STD_DEV
        );

        // Instantiate the pose estimators for each camera
        camera1PoseEstimator = new PhotonPoseEstimator(
            AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded), // Field selection
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, // Strategy selection (look at docs for details)
            ROBOT_TO_CAMERA1); // Camera offset from robot
        
        // camera2PoseEstimator = new PhotonPoseEstimator(
        //     AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded),
        //     PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        //     ROBOT_TO_CAMERA2);

        // camera3PoseEstimator = new PhotonPoseEstimator(
        //     AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded),
        //     PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        //     ROBOT_TO_CAMERA3);

        // camera4PoseEstimator = new PhotonPoseEstimator(
        //     AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded),
        //     PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        //     ROBOT_TO_CAMERA4);        
    }
    
    /**
     * <p> Updates the pose estimators. Calling this outside of robot periodic is unnecesary.
     * <p> Do not run updateUnreadResults() beforehand as it will remove all unread results
     */
    public void updatePoseEstimators() {
        /*
         * Limelight & encoder pose estimation
         */
        // Backup pipeline, so it can be reverted later
        // Change pipeline to allTags so we can use them all 
        //int oldPipeline = (int) LimelightHelpers.getCurrentPipelineIndex("limelight");
        //LimelightHelpers.setPipelineIndex("limelight", 0);

        SwerveModulePosition[] currentPosition = drive.getModulePositions();
        Rotation2d currentRotation = new Rotation2d(drive.getYawRadians());
        
        // Encoder Estimator
        encoderEstimator.update(currentRotation, currentPosition);

        // Update vision estimator with encoder data
        aprilTagsEstimator.update(currentRotation, currentPosition);

        camera1Results = camera1.getAllUnreadResults();
        cameraResults = camera1Results;

        if (camera1Results == null) {
            return;
        }

        if (!camera1Results.isEmpty()) { // Has the camera processed any new results?
            c1noResultsCount = 0;
            latestResult = camera1Results.get(camera1Results.size() - 1);

            if (camera1Results.get(camera1Results.size() - 1).hasTargets()) { // Does the camera see any targets?
                PhotonPipelineResult newResult = camera1Results.get(camera1Results.size() - 1); // Getting newest result
                
                // Checking if the camera sees enough apriltags while moving slow enough to get a good position estimate
                if ((newResult.targets.size() >= REQUIRED_APRILTAGS) && (drive.getYawRateDegrees() <= MAX_YAW_RATE_DEGREES)) {
                    camera1Pose3d = camera1PoseEstimator.update(newResult).get(); // Grab estimated Pose3d from camera
    
                    // DogLog.log("Odometry/purePhotonPose", camera1Pose3d.estimatedPose);
                    // DogLog.log("Odometry/camera1/targetAmbiguity", camera1Pose3d.targetsUsed.get(0).poseAmbiguity);
    
                    // Add vision data to estimator
                    aprilTagsEstimator.addVisionMeasurement(camera1Pose3d.estimatedPose.toPose2d(), // Estimated pose
                                                            camera1Pose3d.timestampSeconds, // Time of sample
                                                            // Standard deviation (inaccuracy) of the camera
                                                            // camera1.getCameraMatrix().get().extractColumnVector(0)
                                                            camera1.getCameraMatrix().orElse(new Matrix<N3, N3>(new SimpleMatrix(3, 3))).extractColumnVector(0));
                }
            }
        }

        /*
        else if (c1noResultsCount <= 3) { // Camera hasn't processed any results within the past 3 or less loops
            c1noResultsCount++;
        }
        else { // Camera hasn't processed any results within the last 4 or more loops; something is wrong
            c1noResultsCount++;
            latestResult = null;
            System.err.println("ERROR 418: camera1: I am a teapot (" + noResultsCount + ")");
        }
        */

        // Repeat above code for each camera
        /*
        camera2Results = camera2.getAllUnreadResults();

        if (camera2Results == null) { // man idk
            return;
        }

        if (!camera2Results.isEmpty()) { // Has the camera processed any new results?
            c2noResultsCount = 0;
            // latestResult = camera2Results.get(camera2Results.size() - 1);

            if (camera2Results.get(camera2Results.size() - 1).hasTargets()) { // Does the camera see any targets?
                PhotonPipelineResult newResult = camera2Results.get(camera2Results.size() - 1); // Getting newest result
                
                // Checking if the camera sees enough apriltags while moving slow enough to get a good position estimate
                if ((newResult.targets.size() >= REQUIRED_APRILTAGS) && (drive.getYawRate() <= MAX_YAW_RATE)) {
                    camera2Pose3d = camera2PoseEstimator.update(newResult).get(); // Grab estimated Pose3d from camera
    
                    // DogLog.log("Odometry/purePhotonPose", camera2Pose3d.estimatedPose);
                    DogLog.log("Odometry/camera2/targetAmbiguity", camera2Pose3d.targetsUsed.get(0).poseAmbiguity);
    
                    // Add vision data to estimator
                    aprilTagsEstimator.addVisionMeasurement(camera2Pose3d.estimatedPose.toPose2d(), // Estimated pose
                                                            camera2Pose3d.timestampSeconds, // Time of sample
                                                            // Standard deviation (inaccuracy) of the camera
                                                            // camera2.getCameraMatrix().get().extractColumnVector(0)
                                                            camera2.getCameraMatrix().orElse(new Matrix<N3, N3>(new SimpleMatrix(3, 3))).extractColumnVector(0));
                }
            }
        }
        else if (c2noResultsCount <= 3) { // Camera hasn't processed any results within the past 3 or less loops
            c2noResultsCount++;
        }
        else { // Camera hasn't processed any results within the last 4 or more loops; something is wrong
            c2noResultsCount++;
            // latestResult = null;
            System.err.println("ERROR 418: camera2: I am a teapot (" + noResultsCount + ")");
        }
        
        /*
        camera3Results = camera3.getAllUnreadResults();

        if (camera3Results == null) { // man idk
            return;
        }

        if (!camera3Results.isEmpty()) { // Has the camera processed any new results?
            c3noResultsCount = 0;
            // latestResult = camera3Results.get(camera3Results.size() - 1);

            if (camera3Results.get(camera3Results.size() - 1).hasTargets()) { // Does the camera see any targets?
                PhotonPipelineResult newResult = camera3Results.get(camera3Results.size() - 1); // Getting newest result
                
                // Checking if the camera sees enough apriltags while moving slow enough to get a good position estimate
                if ((newResult.targets.size() >= REQUIRED_APRILTAGS) && (drive.getYawRate() <= MAX_YAW_RATE)) {
                    camera3Pose3d = camera3PoseEstimator.update(newResult).get(); // Grab estimated Pose3d from camera
    
                    // DogLog.log("Odometry/purePhotonPose", camera3Pose3d.estimatedPose);
                    DogLog.log("Odometry/camera3/targetAmbiguity", camera3Pose3d.targetsUsed.get(0).poseAmbiguity);
    
                    // Add vision data to estimator
                    aprilTagsEstimator.addVisionMeasurement(camera3Pose3d.estimatedPose.toPose3d(), // Estimated pose
                                                            camera3Pose3d.timestampSeconds, // Time of sample
                                                            // Standard deviation (inaccuracy) of the camera
                                                            // camera3.getCameraMatrix().get().extractColumnVector(0)
                                                            camera3.getCameraMatrix().orElse(new Matrix<N3, N3>(new SimpleMatrix(3, 3))).extractColumnVector(0));
                }
            }
        }
        else if (c3noResultsCount <= 3) { // Camera hasn't processed any results within the past 3 or less loops
            c3noResultsCount++;
        }
        else { // Camera hasn't processed any results within the last 4 or more loops; something is wrong
            c3noResultsCount++;
            // latestResult = null;
            System.err.println("ERROR 418: camera3: I am a teapot (" + noResultsCount + ")");
        }

        /*
        camera4Results = camera4.getAllUnreadResults();

        if (camera4Results == null) { // man idk
            return;
        }

        if (!camera4Results.isEmpty()) { // Has the camera processed any new results?
            c4noResultsCount = 0;
            // latestResult = camera4Results.get(camera4Results.size() - 1);

            if (camera4Results.get(camera4Results.size() - 1).hasTargets()) { // Does the camera see any targets?
                PhotonPipelineResult newResult = camera4Results.get(camera4Results.size() - 1); // Getting newest result
                
                // Checking if the camera sees enough apriltags while moving slow enough to get a good position estimate
                if ((newResult.targets.size() >= REQUIRED_APRILTAGS) && (drive.getYawRate() <= MAX_YAW_RATE)) {
                    camera4Pose3d = camera4PoseEstimator.update(newResult).get(); // Grab estimated Pose3d from camera
    
                    // DogLog.log("Odometry/purePhotonPose", camera4Pose3d.estimatedPose);
                    DogLog.log("Odometry/camera4/targetAmbiguity", camera4Pose3d.targetsUsed.get(0).poseAmbiguity);
    
                    // Add vision data to estimator
                    aprilTagsEstimator.addVisionMeasurement(camera4Pose3d.estimatedPose.toPose4d(), // Estimated pose
                                                            camera4Pose3d.timestampSeconds, // Time of sample
                                                            // Standard deviation (inaccuracy) of the camera
                                                            // camera4.getCameraMatrix().get().extractColumnVector(0)
                                                            camera4.getCameraMatrix().orElse(new Matrix<N3, N3>(new SimpleMatrix(3, 3))).extractColumnVector(0));
                }
            }
        }
        else if (c4noResultsCount <= 3) { // Camera hasn't processed any results within the past 3 or less loops
            c4noResultsCount++;
        }
        else { // Camera hasn't processed any results within the last 4 or more loops; something is wrong
            c4noResultsCount++;
            // latestResult = null;
            System.err.println("ERROR 418: camera4: I am a teapot (" + noResultsCount + ")");
        }
        */

        lastPose = currPose;
        currPose = getPose();
    }
 
    /**
     * </p> Gets the current encoder estimated position.
     * 
     * @return The estimated Pose. (in meters)
     */
    public Pose2d getEncoderPose() {
        return encoderEstimator.getPoseMeters();
    }

    /**
     * </p> Gets the current AprilTag-assisted field position.
     *      If no tags are seen or the Orange PIs haven't produced a new result, relies on encoder pose.
     * </p> Refer to the WPILib docs for specifics on field-based odometry.
     * 
     * @return The estimated Pose. (in meters)
     */
    public Pose2d getPose() {
        return aprilTagsEstimator.getEstimatedPosition();
    }

    /**
     * </p> Resets the estimators to a given pose.
     * </p> Gyro angle and swerve module positions don't have to be reset beforehand
     *      as the estimators automatically creates offsets.
     * 
     * @param newPose The Pose2d to reset to.
    */
    public void reset(Pose2d newPose) {
        SwerveModulePosition[] currentPosition = drive.getModulePositions();
        Rotation2d gyro = new Rotation2d(drive.getYawRadians());

        encoderEstimator.resetPosition(
            gyro,
            currentPosition,
            newPose
        );

        aprilTagsEstimator.resetPosition(
            gyro, 
            currentPosition, 
            newPose
        );
    }

    /**
     * @return
     * <p> Distance from the camera to the nearest/best target. 
     * <p> X = forward, Y = left, Z = up. Distance is measured in meters.
     * <p> Rotation (when measured in degrees) stretches from -180 to 180, when the tag is parallel to the camera the angle is 0.
     */
    public Transform3d getAprilTagDistanceToCameraMetric() {
        if (latestResult == null) {
            return null;
        }

        if (!latestResult.hasTargets()) {
            return null;
        }

        Transform3d nonRotated = latestResult.getBestTarget().getBestCameraToTarget().inverse();

        return new Transform3d(
            nonRotated.getTranslation(), nonRotated.getRotation().rotateBy(new Rotation3d(0, 0, Units.degreesToRadians(180)))
        );
    }

    /**
     * @return
     * <p> Distance from the camera to the nearest/best target. 
     * <p> X = forward, Y = left, Z = up. Distance is measured in inches.
     * <p> Rotation (when measured in degrees) stretches from -180 to 180, when the tag is parallel to the camera the angle is 0.
     */
    public Transform3d getAprilTagDistanceToCamera() {
        Transform3d metricTransform = getAprilTagDistanceToCameraMetric();

        if (metricTransform == null) {
            return null;
        }

        return new Transform3d((Units.metersToInches(metricTransform.getX())),
                               (Units.metersToInches(metricTransform.getY())),
                               (Units.metersToInches(metricTransform.getZ())),
                                metricTransform.getRotation());
    }

    /**
     * @return
     * Distance from the robot to the nearest/best target (camera POV).
     * X = forward, Y = left, Z = up. Distance is measured in meters.
     * Rotation (when measured from degrees) stretches from -180 to 180, when the tag is parallel to the camera the angle is 0.
     */
    public Transform3d getAprilTagDistanceMetric() {
        if (getAprilTagDistanceToCameraMetric() == null) {
            return null;
        }

        return getAprilTagDistanceToCameraMetric().plus(ROBOT_TO_CAMERA1.inverse());
    }

    /**
     * @return
     * Distance from the robot to the nearest/best target (camera POV).
     * X = forward, Y = left, Z = up. Distance is measured in inches.
     * Rotation (when measured from degrees) stretches from -180 to 180, when the tag is parallel to the camera the angle is 0.
     */
    public Transform3d getAprilTagDistance() {
        if (getAprilTagDistanceToCamera() == null) {
            return null;
        }

        return getAprilTagDistanceToCamera().plus(ROBOT_TO_CAMERA1.inverse());
    }

    /**
     * @param fieldRelative
     * @return
     */
    public Transform2d getVelocity() {
        Transform2d velocityMeters = currPose.minus(lastPose);

        return velocityMeters;
    }

    /**
     * @return
     * The rotation of the robot relative to the tag (0 is parallel to the tag & positive is clockwise)
     */
    public Rotation2d robotToTagRotation() {
        if (getAprilTagDistanceMetric() == null) {
            return null;
        }

        return getAprilTagDistanceMetric().getRotation().toRotation2d();
    }

    /**
     * This should be tested at some point (if used)
     * @return
     * The rotation of the robot relative to the tag (0 is pointing at the tag & positive is clockwise)
     */
    public Rotation2d robotPointingToTagRotation() {
        if (getAprilTagDistance() == null) {
            return null;
        }

        // photonVision doesn't give this as a function (unlike the limelight) and we need
        // to calculate it manually via trigonometry
        double theta = Math.atan2(getAprilTagDistance().getY(), getAprilTagDistance().getX());

        return new Rotation2d(theta);
    }

    /**
     * Gets every tag seen by each camera. May contain duplicates of the same tag.
     * @return A list of each tag seen by each camera.
     */
    public List<PhotonTrackedTarget> getAllTags() {
        ArrayList<PhotonTrackedTarget> allTags = new ArrayList<PhotonTrackedTarget>();
        
        allTags.addAll(camera1Results.get(camera1Results.size() - 1).getTargets());
        // allTags.addAll(camera2Results.get(camera2Results.size() - 1).getTargets());
        // allTags.addAll(camera4results.get(camera4Results.size() - 1).getTargets());
        // allTags.addAll(camera3results.get(camera3Results.size() - 1).getTargets());

        return (List<PhotonTrackedTarget>) allTags;
    }

    /**
     * @return
     * The fiducial ID of the primary april tag.
     */
    public int primaryTagID() {
        return latestResult.getBestTarget().getFiducialId();
    }

    /**
     * @return
     * How many april tags the camera sees.
     */
    public int targetCount() {
        return latestResult.getTargets().size();
    }

    /**
     * @return
     * Whether the camera sees any targets.
     */
    public boolean hasTargets() {
        return latestResult.hasTargets();
    }

    /*******************************************************************************************
     *
     *                                     HELPER FUNCTIONS
     * 
     *******************************************************************************************/

    /*******************************************************************************************
     *
     *                                      TEST FUNCTIONS
     * 
     *******************************************************************************************/
    /**
     * <p> Prints the current estimated pose from the encoders
    */
     public void printEncoderEstimatorPose() {
        printPose(getEncoderPose());
    }
    /**
     * <p> Prints the current estimated pose from the AprilTags
    */
    public void printAprilTagEstimatorPose() {
        printPose(getPose());
    }
    /**
     * <p> Prints the given pose
    */
    public void printPose(Pose2d pose) {
        Rotation2d rotation = pose.getRotation();

        System.out.println(
            "Field Position: (" + pose.getX() + ", " + pose.getY() + ")\n" +
            "Field Rotation: (" + rotation.getDegrees() + "deg, " + rotation.getRadians() + "rad"
        );
    }

    public void logPeriodic() {
        Logger.logStruct("Odometry/RobotPose", getPose());

        /*
        if (cameraResults == null || cameraResults.isEmpty()) {
            DogLog.log("Odometry/RobotPose", getAprilTagsPose());
            return;
        }

        if (!cameraResults.get(camera1Results.size() - 1).hasTargets()) {
            DogLog.log("Odometry/RobotPose", getAprilTagsPose());
            return;
        }

        DogLog.log("Odometry/RobotPose", getAprilTagsPose());
        //DogLog.log("Odometry/AprilTag/RobotToPrimaryTag", getAprilTagDistance());
        //DogLog.log("Odometry/AprilTag/CameraToPrimaryTag", getAprilTagDistanceToCamera());
        //DogLog.log("Odometry/AprilTag/PrimaryTagID", cameraResults.get(camera1Results.size() - 1).getBestTarget().fiducialId);
        //DogLog.log("Odometry/AprilTag/VisibleTags", cameraResults.get(camera1Results.size() - 1).getTargets().size());
        */
    }
}
