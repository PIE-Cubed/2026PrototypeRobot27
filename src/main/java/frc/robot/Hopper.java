package frc.robot;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class Hopper {
    private final int MAX_BELT_STALL_LIMIT = 20;
    private final int MAX_BELT_FREE_LIMIT  = 60;
    private final int INDEX_MOTOR_ID = 33;
    private final int BELT_MOTOR_ID  = 32;

    private SparkBase       indexMotor       = new SparkMax(INDEX_MOTOR_ID, MotorType.kBrushless);
    private SparkBaseConfig indexMotorConfig = new SparkMaxConfig();

    private SparkBase       beltMotor       = new SparkMax(BELT_MOTOR_ID, MotorType.kBrushless);
    private SparkBaseConfig beltMotorConfig = new SparkMaxConfig();

    public Hopper() {
        indexMotorConfig.inverted(false)
                        .smartCurrentLimit(Robot.NEO_CURRENT_LIMIT)
                        .idleMode(IdleMode.kBrake);

        indexMotor.configure(indexMotorConfig, 
                             ResetMode.kNoResetSafeParameters, 
                             PersistMode.kPersistParameters);
                             
        beltMotorConfig.inverted(false)
                       .smartCurrentLimit(MAX_BELT_STALL_LIMIT, MAX_BELT_FREE_LIMIT) 
                       .idleMode(IdleMode.kBrake);

        beltMotor.configure(indexMotorConfig, 
                            ResetMode.kNoResetSafeParameters, 
                            PersistMode.kPersistParameters);

        // TODO: Figure out a good speed for this later.
        SmartDashboard.putNumber("Hopper/MotorPower", 1);
    }

    public void indexFuel() {
        indexMotor.set(SmartDashboard.getNumber("Hopper/MotorPower", 0));
        beltMotor.set(1);
    }

    public void stopMotors() {
        indexMotor.stopMotor();
        beltMotor.stopMotor();
    }
}
