package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;
import java.util.Optional;

// TODO: Make ALL of this not stupid
public class AllianceUtil {
    public static boolean isRedAlliance() {
        Optional<Alliance> alliance = DriverStation.getAlliance();

        return alliance.isPresent() && alliance.get() == Alliance.Red;
    }

    public static int getShift(double time) {
        if (time <= 130 && time > 105) return 1;
        if (time <= 105 && time > 80) return 2;
        if (time <= 80 && time > 55) return 3;
        if (time <= 55 && time > 30) return 4;
        return -1;
    }

    public static boolean isOurShift(int shift) {
        boolean oddShift = (shift % 2 == 1);

        String gameData = DriverStation.getGameSpecificMessage();
        if (gameData.isEmpty()) {
            return true;
        }

        boolean isRed = isRedAlliance();

        if (gameData.charAt(0) == 'R') {
            return oddShift ? !isRed : isRed;
        } 
        else {
            return oddShift ? isRed : !isRed;
        }
    }

    public static double timeUntilShift(double time) {
        

        if (DriverStation.isAutonomous() || time > 140 || (time <= 140 && time > 130) || time <= 30) {
            return 0;
        }

        String gameData = DriverStation.getGameSpecificMessage();
        if (gameData.isEmpty()) {
            return 0;
        }

        int currentShift = getShift(time);
        if (currentShift == -1) {
            return 0;
        }

        switch (currentShift) {
            case 1:
                return time - 105;
            case 2:
                return time - 80;
            case 3:
                return time - 55;
            case 4:
                return 0;

            default:
                return 0;
        }
    }

    public static double getMatchTime() {
        return DriverStation.getMatchTime();
    }
}
