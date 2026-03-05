package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig; //C: this is mass importation


public class DriveSubsystem extends SubsystemBase {

  private final SparkMax m_leftLeader = new SparkMax(4, MotorType.kBrushless); //C: defines sparkmax 4 as front left, as a leader to let us know it's meant to receive input
  private final SparkMax m_leftFollower = new SparkMax(3, MotorType.kBrushless); //C: defines sparkmax 3 as back left, as a follower to let us know it matches its leaders' input
  private final SparkMax m_rightLeader = new SparkMax(2, MotorType.kBrushless); //C: ditto of line 11 but for the right and uses sparkmax 2
  private final SparkMax m_rightFollower = new SparkMax(1, MotorType.kBrushless); //C: ditto of line 12 but for the right and uses sparkmax 1

  private Gyroscope m_gyroscope;
  private Pneumatics m_pneumatics;

  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftLeader, m_rightLeader);

  private final PIDController m_pid = new PIDController(0.03, 0.01, 0);

  private final DifferentialDriveOdometry m_odometry;

  private final RelativeEncoder m_leftEncoder = m_leftLeader.getEncoder();
  private final RelativeEncoder m_rightEncoder = m_rightLeader.getEncoder();

  private final DifferentialDrivetrainSim m_driveSim = new DifferentialDrivetrainSim(
    DCMotor.getNEO(2),
    7.29,
    4.22,
    Units.lbsToKilograms(60),
    Units.inchesToMeters(3),
    Units.inchesToMeters(25),
    null
  );

  private final edu.wpi.first.wpilibj.smartdashboard.Field2d m_Field2d = new Field2d();

  // private final double positionConversion = (Units.inchesToMeters(6) * Math.PI) / 7.29;

  private final double highGearThreshold = 3.5;   // Meters per second (Tune these!)
  private final double lowGearThreshold = 1.0;

  private final double lowGearRatio = 7.29;
  private final double highGearRatio = 2.43;

  private boolean isAutoShiftEnabled = true;

  private int autoShiftTimer;

  /**
   * The main class to drive the robot. Used in {@link frc.robot.RobotContainer RobotContainer}.
   *
   * @param gyro This is a parameter that initializes the gyroscope for DriveSubsystem based on the RobotContainer
   * @param pneumatics This is a parameter that takes in the pneumatics from RobotContainer
  */

  // ! This drives the entire robot. Make sure your changes work in Sim before you edit!

  // DONE: Add Odometry

  // TESTING: Odometry logging in AdvantageScope

  // TODO: PathPlanner
  // TODO: SysID
  // TODO: Make PhotonVision
  // TODO: Add PhotonVision info to logs

  public DriveSubsystem(Gyroscope gyro, Pneumatics pneumatics) {
    this.m_gyroscope = gyro;
    this.m_pneumatics = pneumatics;

    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);

    this.m_odometry = new DifferentialDriveOdometry(
      Rotation2d.fromDegrees(m_gyroscope.getHeading()),
      0.0,
      0.0
    );

    SparkMaxConfig leftConfig = new SparkMaxConfig();
    SparkMaxConfig leftLeaderConfig = new SparkMaxConfig(); //C: preparing to pair the left side
    SparkMaxConfig rightLeaderConfig = new SparkMaxConfig();
    SparkMaxConfig rightConfig = new SparkMaxConfig(); //C: ditto for right

    // Configure followers to follow their respective leaders
    leftConfig.follow(4); //C: now the code recognizes CanID 4 (the m_leftLeader) as a leader
    rightConfig.follow(2); //C: same with the right, numbers respective

    // Invert one side so positive power moves the robot forward
    rightConfig.inverted(true); //C: not much i can explain here when the above comment said it all already
    rightLeaderConfig.inverted(true);

    // leftLeaderConfig.encoder.positionConversionFactor(positionConversion);
    // rightLeaderConfig.encoder.positionConversionFactor(positionConversion);

    // 3. Apply configurations
    m_leftLeader.configure(leftLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_leftFollower.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightLeader.configure(rightLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightFollower.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    ShuffleboardTab m_tab = Shuffleboard.getTab("Drive System");

    m_tab.add("Drive Train", m_drive)
      .withWidget(BuiltInWidgets.kDifferentialDrive)
      .withSize(4, 3)
      .withPosition(0, 0);

    m_tab.add("180 PID", m_pid)
      .withWidget(BuiltInWidgets.kPIDCommand);

    m_tab.add("Field", m_Field2d)
      .withWidget(BuiltInWidgets.kField)
      .withSize(4, 6)
      .withPosition(4, 0);

    m_tab.add("Toggle Auto Shift", toggleAutoShift())
      .withWidget(BuiltInWidgets.kCommand)
      .withPosition(5, 0)
      .withSize(2, 1);

    m_tab.add("Auto Shift On", isAutoShiftEnabled)
      .withWidget(BuiltInWidgets.kBooleanBox)
      .getEntry();

    m_pid.enableContinuousInput(-180, 180);
    m_pid.setTolerance(2);
  }

  /**
   * This is the script that tells the robot to drive based on controller inputs
   *
   * @param speed The speed applied to the motors to drive forward
   * @param rotation The changes to voltage to apply to cause a rotation determined based on the stick
   *
   * @see edu.wpi.first.wpilibj.drive.DifferentialDrive#arcadeDrive(double, double) arcadeDrive()
   *
   * @return Information on how to apply voltage to motors to drive in specified way
  */

  // ! Do not remove this! The robot will not drive!

  public void arcadeDrive(double speed, double rotation) {
    m_drive.arcadeDrive(speed, rotation);
  }

  /**
   * A PID that causes the robot to turn 180 degrees around
   *
   * @implNote Check the {@code m_pid} variable in {@link DriveSubsystem} to change PID values
   *
   * @return A command that tells the robot to calculate a target then turns to the target
  */

  // ! To change values, check Drive Subsytem

  public Command turn180() {
    return Commands.sequence(
      runOnce(
        () -> {
          double target = MathUtil.inputModulus(m_gyroscope.getHeading() + 180, -180, 180);
          m_pid.setSetpoint(target);
        }
      ),
      run(
        () -> {
          double rotationSpeed = m_pid.calculate(m_gyroscope.getHeading());
          rotationSpeed = Math.max(-0.75, Math.min(0.75, rotationSpeed));
          this.arcadeDrive(0, rotationSpeed);
        }
      ).until(
        m_pid::atSetpoint
      )
    ).finallyDo(
      () -> CommandScheduler.getInstance().cancel(this.turn180())
    );
  }

  @Override
  public void simulationPeriodic() {
    double currentRatio = m_pneumatics.isHighGear() ? highGearRatio : lowGearRatio;

    m_driveSim.setCurrentGearing(currentRatio);

    m_driveSim.setInputs(m_leftLeader.get() * 12.0, m_rightLeader.get() * 12.0);
    m_driveSim.update(0.020);

    m_leftEncoder.setPosition(m_driveSim.getLeftPositionMeters());
    m_rightEncoder.setPosition(m_driveSim.getRightPositionMeters());
    m_gyroscope.setSimHeading(m_driveSim.getHeading().getDegrees());

    Logger.recordOutput("Drive/RobotPose", m_driveSim.getPose());
  }

  /**
   * Resets the simulation rotation to allow reseting the gyro info to work
   *
   * @return A command that sets the rotation of the {@link edu.wpi.first.math.geometry.Pose2d#Pose2d() Pose2D} to 0 degrees
   */

  public Command resetSimPose() {
      return runOnce(() -> {
        if (Robot.isSimulation()) {
            var currentPose = m_driveSim.getPose();

            var resetPose = new edu.wpi.first.math.geometry.Pose2d(
                currentPose.getTranslation(), 
                new edu.wpi.first.math.geometry.Rotation2d(0)
            );

            m_driveSim.setPose(resetPose);
        }
    });
  }

  /**
   * Updates the encoders in sim mode to speed up the robot in high gear
   *
   * @param ratio The ratio previously defined in {@link DriveSubsystem}
   */
  private void updateEncoderConversion(double ratio) {
    // double newPosFactor = (Units.inchesToMeters(6) * Math.PI) / ratio;
    
    // Create a temporary config to apply the change
    SparkMaxConfig config = new SparkMaxConfig();
    // config.encoder.positionConversionFactor(newPosFactor);
    // config.encoder.velocityConversionFactor(newPosFactor / 60.0);

    m_leftLeader.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    m_rightLeader.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /**
   * A command to turn on and off auto shift
   *
   * @see #autoShift()
   *
   * @return A command that swaps {@link #isAutoShiftEnabled} and logs it
   */
  public Command toggleAutoShift() {
    return runOnce(() -> {
        isAutoShiftEnabled = !isAutoShiftEnabled;
        Logger.recordOutput("Drive/AutoShiftEnabled", isAutoShiftEnabled);
    });
  }

  /**
   * Finds the average Meters Per Second of both Spark Maxes then shifts.
   *
   * <ul>
   * <li> <b>Reasons for shifting:</b> </li>
   * <ul>
   * <li> Shift up if speed hits {@link #highGearThreshold} </li>
   * <li> Shift down if speed hits {@link #lowGearThreshold} </li>
   * <li> Shift down if power draw is higher than {@link #highCurrentThreshold}
   * </ul>
   * </ul>
   *
   * @see Pneumatics#toggleSolenoids()
  */
  private void autoShift() {
    if (!isAutoShiftEnabled) return;

    if (autoShiftTimer > 0) {
      autoShiftTimer--;
      return; 
    }

    double currentRatio = m_pneumatics.isHighGear() ? highGearRatio : lowGearRatio;
    double currentVelocityConversion = (Units.inchesToMeters(6) * Math.PI) / (currentRatio * 60.0);

    double leftMpS = m_leftEncoder.getVelocity() * currentVelocityConversion;
    double rightMpS = m_rightEncoder.getVelocity() * currentVelocityConversion;
    double avgVelocity = Math.abs((leftMpS + rightMpS) / 2.0);
    
    boolean currentlyHigh = m_pneumatics.isHighGear();

    if (!currentlyHigh && avgVelocity > highGearThreshold) {
      m_pneumatics.setHighGear(true);
      updateEncoderConversion(highGearRatio);
      autoShiftTimer = 15;
      System.out.println("switched to high" + avgVelocity);
    }
    else if (currentlyHigh && avgVelocity < lowGearThreshold) {
      m_pneumatics.setHighGear(false);
      updateEncoderConversion(lowGearRatio);
      System.out.println("no high gear? :(" + avgVelocity);
      autoShiftTimer = 15;
    }
  }

  @Override
  public void periodic() {
    m_odometry.update(
      Rotation2d.fromDegrees(m_gyroscope.getHeading()),
      m_leftEncoder.getPosition(),
      m_rightEncoder.getPosition()
    );
    autoShift();

    m_Field2d.setRobotPose(m_odometry.getPoseMeters());

    Logger.recordOutput("Drive/Pose", m_odometry.getPoseMeters());
  }
}