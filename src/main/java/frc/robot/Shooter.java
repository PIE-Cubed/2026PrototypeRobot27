package frc.robot;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

public class Shooter {
    private SparkFlex       backspinMotor;
    private SparkBaseConfig backspinMotorConfig;

    private SparkFlex       flywheelMotor;
    private SparkBaseConfig flywheelMotorConfig;

    private RelativeEncoder backspinMotorEncoder;
    private RelativeEncoder flywheelMotorEncoder;

    private PIDController backspinPIDController;
    private PIDController flywheelPIDController;

    // Motor IDs
    private final int BACKSPIN_MOTOR_ID = 21;
    private final int FLYWHEEL_MOTOR_ID = 20;

    // PID Values
    private final double BACKSPIN_P = 0.0055;
    private final double BACKSPIN_I = 0.0;
    private final double BACKSPIN_D = 0.0;

    private final double FLYWHEEL_P = 0.0055;
    private final double FLYWHEEL_I = 0.0;
    private final double FLYWHEEL_D = 0.0;

    private double prevShooterVelocity = 0;
    private double prevShooterVoltage  = 0;

    // top motor runs slower than bottom motor for backspin
    private static final double SHOOTER_MOTOR_DELTA = .1;
    private static final double VELOCITY_TO_VOLT_RATIO = 550;

    public Shooter() {
        backspinMotor       = new SparkFlex(BACKSPIN_MOTOR_ID, MotorType.kBrushless);
        backspinMotorConfig = new SparkFlexConfig();

        flywheelMotor       = new SparkFlex(FLYWHEEL_MOTOR_ID, MotorType.kBrushless);
        flywheelMotorConfig = new SparkFlexConfig();

        // TODO invert motor if needed
        backspinMotorConfig.idleMode(IdleMode.kCoast);
        backspinMotorConfig.inverted(true);

        flywheelMotorConfig.idleMode(IdleMode.kCoast);
        flywheelMotorConfig.inverted(false);

        backspinMotor.configure(backspinMotorConfig, 
                                ResetMode.kNoResetSafeParameters, 
                                PersistMode.kPersistParameters);

        flywheelMotor.configure(flywheelMotorConfig, 
                                ResetMode.kNoResetSafeParameters, 
                                PersistMode.kPersistParameters);

        backspinMotorEncoder = backspinMotor.getEncoder();
        flywheelMotorEncoder = flywheelMotor.getEncoder();

        backspinPIDController = new PIDController(BACKSPIN_P, BACKSPIN_I, BACKSPIN_D);
        flywheelPIDController = new PIDController(FLYWHEEL_P, FLYWHEEL_I, FLYWHEEL_D);
    }

    /**
     * voltage should be -12 to 12
     * @param voltage
     */
    public void setTopMotorVoltage(double voltage) {
        backspinMotor.setVoltage(voltage);
    }

    /**
     * voltage should be -12 to 12
     * @param voltage
     */
    public void setBottomMotorVoltage(double voltage) {
        flywheelMotor.setVoltage(voltage);
    }

    /**
     * speed ....
     * @param speed
     */
    // TODO get conversion factor for speed
    public void setTopTargetSpeed(double speed) {
        
    }

    /**
     * speed ....
     * @param speed
     */
    // TODO get conversion factor for speed
    public void setBottomTargetSpeed(double speed) {

    }
    
    public void setVelocity(double velocity)  {
        double voltage;
        double pidOutput;
    

        if (velocity == prevShooterVelocity){
            voltage = prevShooterVoltage;
        }
        else {
            voltage = velocity/VELOCITY_TO_VOLT_RATIO;
        }
        
        pidOutput = flywheelPIDController.calculate(flywheelMotorEncoder.getVelocity(), velocity);
        voltage = voltage + pidOutput;

        voltage = MathUtil.clamp(voltage, -12.0, 12.0);
        flywheelMotor.setVoltage(voltage);
        if (voltage >= 0.0) {
            backspinMotor.setVoltage(voltage - SHOOTER_MOTOR_DELTA);
        } 
        else {
            backspinMotor.setVoltage(voltage + SHOOTER_MOTOR_DELTA);
        }    
         
        prevShooterVoltage  = voltage;
        prevShooterVelocity = velocity;
        System.out.println("velocity" + flywheelMotorEncoder.getVelocity());
    }

    /******************************************************************************************************
     * 
     * TEST PROGRAMS
     * 
     ******************************************************************************************************/

    public void testVoltageVsVelocity(double voltage) {
        double velocity;

        flywheelMotor.setVoltage(voltage);
        velocity = flywheelMotorEncoder.getVelocity();
        System.out.println("velocity = " + velocity + "    " + voltage);
    
    }
}

/* FOR MOTOR SET FUCNTION
* .1 volts = 606 RPM
* .2 volts = 1240 RPM
* .3 volts = 1866 RPM
* .4 volts = 2500 RPM
* .5 volts = 3140 RPM
* .6 volts = 3783 RPM
* .7 volts =  4408 RPM
* .8 volts =  5012 RPM
* .9 volts = 5600 RPM
* 1 volts =  6175 RPM
* 
* MOTORSETVOLTAGE (More Stable than SET FUNCTOIN)
* volts  RPM    Delta   RPM/Volts
* 1      527            527
* 2      1085   558     542.5
* 3      1642   560     547.3
* 4      2201   559     550.25
* 5      2761   560     552.2
* 6      3322   561     553.67
* 7      3884   562     554.85
* 8      4446   562     555.75
* 9      5006   560     556.22
* 10     5550   544     555
* 11     6090   540     553.64
* MAX 12 6155   065     512.92
*/