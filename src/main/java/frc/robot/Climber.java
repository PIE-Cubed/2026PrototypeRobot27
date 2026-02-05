package frc.robot;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Climber {

    private SparkBase climbMotor;
    private SparkBaseConfig climbMotorConfig;

    // Motor IDs
    private final int CLIMB_MOTOR_ID = 51;

    public Climber() {
        climbMotor = new SparkMax(CLIMB_MOTOR_ID, MotorType.kBrushless);
        climbMotorConfig = new SparkMaxConfig();

        climbMotorConfig.idleMode(IdleMode.kBrake);
        climbMotorConfig.smartCurrentLimit(Robot.NEO_CURRENT_LIMIT);
        climbMotorConfig.inverted(false);

        climbMotor.configure(climbMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    /********************************************************************
     *
     * TEST PROGRAMS
     *
     ********************************************************************/

    public void testMotor(double upPower, double downPower) {
        if (upPower != 0) {
            climbMotor.set(upPower);
        } else if (downPower != 0) {
            climbMotor.set(downPower);
        } else {
            climbMotor.stopMotor();
        }
    }
}
