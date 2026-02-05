// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.Choreo;
import choreo.trajectory.EventMarker;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.AllianceUtil;
import java.util.List;
import java.util.Optional;

/**
 * This class has all the autonomous programs.
 */
public class Auto {

    // Required classes
    private Drive drive;
    private Shooter shooter;
    private Hopper hopper;

    // Class-wide variables
    private final Timer timer = new Timer();
    private Optional<Trajectory<SwerveSample>> currentTrajectory;
    // private Optional<Trajectory<SwerveSample>> currentSplit;
    // private int currentSplitIndex = 1;
    private boolean firstTime = true;
    private int step = 1;

    // Auto-specific variables
    // Outpost
    // private int outpostClimbOutStep = 1;
    // private int outpostClimbDepStep = 1;
    // private int outpostNoClimbStep = 1;

    // Center
    // private int centerOutClimbOutStep = 1;
    // private int centerDepClimbOutStep = 1;
    // private int centerOutClimbDepStep = 1;
    // private int centerDepClimbDepStep = 1;
    // private int centerOutNoClimbStep = 1;
    // private int centerDepNoClimbStep = 1;

    // Depot
    // private int depotClimbOutStep = 1;
    // private int depotClimbDepStep = 1;
    // private int depotNoClimbStep = 1;

    public Auto(Drive drive, Shooter shooter, Hopper hopper) {
        this.drive = drive;
    }

    public void resetAuto() {
        firstTime = true;
    }

    ///////////////////
    /* OUTPOST AUTOS */
    ///////////////////

    public int outpostClimbOut() {
        return Robot.CONT;
    }

    public int outpostClimbDep() {
        return Robot.CONT;
    }

    public int outpostNoClimb() {
        return Robot.CONT;
    }

    //////////////////
    /* CENTER AUTOS */
    //////////////////

    public int centerDepClimbDep() {
        return Robot.CONT;
    }

    public int centerDepClimbOut() {
        return Robot.CONT;
    }

    public int centerOutClimbDep() {
        return Robot.CONT;
    }

    public int centerOutClimbOut() {
        if (firstTime) {
            firstTime = false;
            step = 1;
            restartTimer();
        }

        int status = Robot.CONT;
        int pathStatus = choreoPathFollower("centerOCO");

        switch (step) {
            case 1:
                status = atMarker("startShoot");
                break;
            case 2:
                // add other stuff after
                break;
            default:
                return Robot.DONE;
        }

        if (status == Robot.DONE) {
            step++;
        }

        return Robot.CONT;
    }

    public int centerDepNoClimb() {
        return Robot.CONT;
    }

    public int centerOutNoClimb() {
        return Robot.CONT;
    }

    /////////////////
    /* DEPOT AUTOS */
    /////////////////

    public int depotClimbDep() {
        return Robot.CONT;
    }

    public int depotClimbOut() {
        return Robot.CONT;
    }

    public int depotNoClimb() {
        return Robot.CONT;
    }

    /////////////
    /* HELPERS */
    /////////////

    public int choreoPathFollower(String trajectoryFileName) {
        if (currentTrajectory.get().name() != trajectoryFileName) {
            currentTrajectory = Choreo.loadTrajectory(trajectoryFileName);
        }

        // if (currentSplitIndex != splitIndex) {
        //     currentSplit = currentTrajectory.get().getSplit(splitIndex);
        //     currentSplitIndex = splitIndex;
        // }

        if (currentTrajectory.isPresent() == true) {
            Optional<SwerveSample> sample = currentTrajectory.get().sampleAt(timer.get(), AllianceUtil.isRedAlliance());

            if (sample.isPresent() == true) {
                //System.out.println("Sample time: " + sample.get().getTimestamp());
                drive.followSample(sample.get());
                //choreoPose2d = sample.get().getPose();
                // DogLog.log("Auto/choreoTargetPose", sample.get().getPose());
            }
        } else { // there isn't a trajectory
            System.err.println("ERROR 404: Trajectory not found");
            return Robot.FAIL;
        }

        return Robot.CONT;
    }

    /**
     * Checks whether the choreo path is at a marker.
     * Uses recursion, so try not to run trajectories with more than a couple markers that have the same name.
     * @param markerName The name of the marker.
     * @return True if at a marker, false if not.
     */
    public int atMarker(String markerName) {
        List<EventMarker> events = currentTrajectory.get().getEvents(markerName);

        for (int i = 0; i < events.size(); i++) {
            if (MathUtil.isNear(events.get(i).timestamp, timer.get(), 0.03)) return Robot.DONE;
        }

        return Robot.CONT;
    }

    public void restartTimer() {
        timer.restart();
    }
}
