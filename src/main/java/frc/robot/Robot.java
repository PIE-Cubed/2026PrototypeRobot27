package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.AllianceUtil;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  public static final int FAIL = -1;
  public static final int PASS = 1;
  public static final int DONE = 2;
  public static final int CONT = 3;

  public static final int NEO_CURRENT_LIMIT = 60;
  public static final int NEO_550_CURRENT_LIMIT = 30;
  public static final int VORTEX_CURRENT_LIMIT = 80;

  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private PowerDistribution pdh;

  private final Field2d field2d = new Field2d();

  private Controls controls;
  private Drive    drive;
  private Odometry odometry;
  private Hopper   hopper;
  private Shooter  shooter;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // TODO: ADD AUTOS
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    pdh = new PowerDistribution(1, ModuleType.kRev);

    controls = new Controls();
    drive    = new Drive();
    //odometry = new Odometry(drive);
    hopper   = new Hopper();
    shooter  = new Shooter();

    SmartDashboard.putNumber("Shooter/FlywheelTargetRPM", 0);
    SmartDashboard.putNumber("Shooter/BackspinTargetRPM", 0);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    //odometry.updatePoseEstimators();

    //SmartDashboard.putNumber("Voltage", pdh.getVoltage());

    //field2d.setRobotPose(Odometry.getPose());

    SmartDashboard.putData("Field", field2d);

    // TODO: Make this not stupid
    double time = AllianceUtil.getMatchTime();

    SmartDashboard.putNumber("Match Time", time);
    SmartDashboard.putNumber("Time Until Shift", AllianceUtil.timeUntilShift(time));
    SmartDashboard.putBoolean("Hub Active", AllianceUtil.isOurShift(AllianceUtil.getShift(time)));
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    wheelControl();
    shooterControl();

    if (controls.getAButton()) {
      shooter.setBottomMotorVoltage(3);
      shooter.setTopMotorVoltage(3);
    }
    else {
      shooter.stopMotors();
    }

    /*
    SmartDashboard.putBoolean(
      "Shooter/FlywheelAtTargetRPM",
      shooter.setFlywheelRPM(SmartDashboard.getNumber("Shooter/FlywheelTargetRPM", 0)) == DONE
    );

    SmartDashboard.putBoolean(
      "Shooter/BackspinAtTargetRPM", 
      shooter.setBackspinRPM(SmartDashboard.getNumber("Shooter/BackspinTargetRPM", 0)) == DONE
    );
    */
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  private void wheelControl() {
    boolean resetGyro = controls.resetGyro();
    boolean fieldDrive = controls.getFieldDrive();
    boolean lockWheels = controls.getWheelLock();

    double forwardPowerFwdPos = controls.getForwardPowerFwdPositive();
    double strafePowerLeftPos = controls.getStrafePowerLeftPositive();
    double rotatePowerCcwPos = controls.getRotatePowerCcwPositive();

    if (lockWheels) {
      drive.lockWheels();
    } 
    else {
      drive.teleopDrive(forwardPowerFwdPos, strafePowerLeftPos, rotatePowerCcwPos, fieldDrive);
    }

    if (resetGyro) {
      drive.resetGyro();
    }
  }

  private void shooterControl() {
    boolean shootButton = controls.getShootButton();

    if (shootButton == true) {
      hopper.indexFuel();
    } 
    else {
      hopper.stopMotors();
    }
  }
}
