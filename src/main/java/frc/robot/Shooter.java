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
import edu.wpi.first.units.measure.Voltage;




public class Shooter {
    private SparkFlex       topMotor;
    private SparkBaseConfig topMotorConfig;

    private SparkFlex       bottomMotor;
    private SparkBaseConfig bottomMotorConfig;

    private RelativeEncoder topMotorEncoder;
    private RelativeEncoder bottomMotorEncoder;

    private PIDController topPIDController;
    private PIDController bottomPIDController;

    // Motor IDs
    private final int TOP_MOTOR_ID = 21;
    private final int BOTTOM_MOTOR_ID = 20;

    // PID Values
    private final double TOP_P = 0.0055;
    private final double TOP_I = 0.0;
    private final double TOP_D = 0.0;

    private final double BOTTOM_P = 0.0055;
    private final double BOTTOM_I = 0.0;
    private final double BOTTOM_D = 0.0;

    private double prevShooterVelocity = 0;
    private double prevShooterVoltage  = 0;


    // top motor runs slower than bottom motor for backspin
    private static final double SHOOTER_MOTOR_DELTA = .1;
    private static final double VELOCITY_TO_VOLT_RATIO = 550;

    public Shooter() {
        topMotor       = new SparkFlex(TOP_MOTOR_ID, MotorType.kBrushless);
        topMotorConfig = new SparkFlexConfig();

        bottomMotor       = new SparkFlex(BOTTOM_MOTOR_ID, MotorType.kBrushless);
        bottomMotorConfig = new SparkFlexConfig();

        // TODO invert motor if needed
        topMotorConfig.idleMode(IdleMode.kCoast);
        topMotorConfig.inverted(true);

        bottomMotorConfig.idleMode(IdleMode.kCoast);
        bottomMotorConfig.inverted(false);

        topMotor.configure(topMotorConfig, 
                           ResetMode.kNoResetSafeParameters, 
                           PersistMode.kPersistParameters);

        bottomMotor.configure(bottomMotorConfig, 
                              ResetMode.kNoResetSafeParameters, 
                              PersistMode.kPersistParameters);

        topMotorEncoder =    topMotor.getEncoder();
        bottomMotorEncoder = bottomMotor.getEncoder();

        topPIDController =    new PIDController(TOP_P, TOP_I, TOP_D);
        bottomPIDController = new PIDController(BOTTOM_P, BOTTOM_I, BOTTOM_D);
    }

    /**
     * voltage should be -12 to 12
     * @param voltage
     */
    public void setTopMotorVoltage(double voltage) {
        topMotor.setVoltage(voltage);
    }

    /**
     * voltage should be -12 to 12
     * @param voltage
     */
    public void setBottomMotorVoltage(double voltage) {
        bottomMotor.setVoltage(voltage);
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
        
        pidOutput = bottomPIDController.calculate(bottomMotorEncoder.getVelocity(), velocity);
        voltage = voltage + pidOutput;

        voltage = MathUtil.clamp(voltage, -12.0, 12.0);
        bottomMotor.setVoltage(voltage);
        if (voltage >= 0.0) {
            topMotor.setVoltage(voltage - SHOOTER_MOTOR_DELTA);
        } 
        else {
            topMotor.setVoltage(voltage + SHOOTER_MOTOR_DELTA);
        }    
         
        prevShooterVoltage  = voltage;
        prevShooterVelocity = velocity;
        System.out.println("velocity" + bottomMotorEncoder.getVelocity());
    }



    /******************************************************************************************************
     * 
     * TEST PROGRAMS
     * 
     ******************************************************************************************************/
        /* FOR MOTOR SET FUCNTION
         .1 volts = 606 RPM
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
     public void testVoltageVsVelocity(double voltage)
    {
        double velocity;



        //bottomMotor.set(voltage);        
        bottomMotor.setVoltage(voltage);
        velocity = bottomMotorEncoder.getVelocity();
        System.out.println("velocity = " + velocity + "    " + voltage);
    
    } 

}
