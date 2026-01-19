package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;

import java.util.Optional;

public class AllianceUtil {
    // this is easier than writing down FieldConstants.TRANSITION_PERIOD_SECONDS every time
    private static final int TELEOP_LENGTH_SECONDS     = FieldConstants.TELEOP_LENGTH_SECONDS;
    private static final int TRANSITION_PERIOD_SECONDS = FieldConstants.TRANSITION_PERIOD_SECONDS;
    private static final int SHIFT_LENGTH_SECONDS      = FieldConstants.SHIFT_LENGTH_SECONDS;
    private static final int ENDGAME_LENGTH_SECONDS    = FieldConstants.ENDGAME_LENGTH_SECONDS;

    public static boolean isRedAlliance() {
        Optional<Alliance> alliance = DriverStation.getAlliance();

        return alliance.isPresent() && alliance.get() == Alliance.Red;
    }

    /**
     * Gets the start time for each period. If input period is 0, returns 140, or the length of teleop.
     * (ex: returns 55 with an input of 4, returns 130 with an input of 1, returns 140 with an input of 0)
     * @param period The period number to return the start time of.
     * @return The start time of a specified period.
     */
    private static int getPeriodTime(int period) {
        period = MathUtil.clamp(period, 0, 4);

        return MathUtil.clamp(
            (TRANSITION_PERIOD_SECONDS - SHIFT_LENGTH_SECONDS) + (period * SHIFT_LENGTH_SECONDS), 0, TELEOP_LENGTH_SECONDS
        );
    }

    public static int getShift(double time) {
        if (time <= getPeriodTime(0) && time > getPeriodTime(1)) return 0;
        if (time <= getPeriodTime(1) && time > getPeriodTime(2)) return 1;
        if (time <= getPeriodTime(2) && time > getPeriodTime(3)) return 2;
        if (time <= getPeriodTime(3) && time > getPeriodTime(4)) return 3;
        if (time <= getPeriodTime(4) && time > ENDGAME_LENGTH_SECONDS)  return 4;
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
        if (DriverStation.isAutonomous() || time > getPeriodTime(1) || time <= ENDGAME_LENGTH_SECONDS) {
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
            case 0:
                return time - getPeriodTime(1);
            case 1:
                return time - getPeriodTime(2);
            case 2:
                return time - getPeriodTime(3);
            case 3:
                return time - getPeriodTime(4);
            case 4:
                return time - ENDGAME_LENGTH_SECONDS;

            default:
                return 0;
        }
    }

    public static double getMatchTime() {
        return DriverStation.getMatchTime();
    }
}
