package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.LTVUnicycleController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import choreo.auto.AutoFactory;
import choreo.trajectory.DifferentialSample;
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
    5.0,
    4.22,
    Units.lbsToKilograms(60),
    Units.inchesToMeters(3),
    Units.inchesToMeters(25),
    null
  );

  private final edu.wpi.first.wpilibj.smartdashboard.Field2d m_Field2d = new Field2d();

  // private final double positionConversion = (Units.inchesToMeters(6) * Math.PI) / 7.29;

  private final double highGearThreshold = 5.0;   // ! Meters per second (Tune these!)
  private final double lowGearThreshold = 1.0;

  private final double lowGearRatio = 20.523724;
  private final double highGearRatio = 9.261941;

  private boolean isAutoShiftEnabled = true;

  private int autoShiftTimer;

  private final LTVUnicycleController controller = new LTVUnicycleController(0.02);

  private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(25));

  private final SysIdRoutine m_sysIdRoutine =
    new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            null,
            null,
            (state) -> Logger.recordOutput("SysId/State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            (voltage) -> {
                m_leftLeader.setVoltage(voltage);
                m_rightLeader.setVoltage(voltage);
                m_drive.feed();
            },

            (SysIdRoutineLog log) -> {
              log.motor("Drivetrain")
               .voltage(
                    edu.wpi.first.units.Units.Volts.of(
                        m_leftLeader.getAppliedOutput()
                            * m_leftLeader.getBusVoltage()
                    )
                )
                .angularPosition(
                  edu.wpi.first.units.Units.Rotations.of(m_leftEncoder.getPosition())
                )
                .angularVelocity(
                  edu.wpi.first.units.Units.RotationsPerSecond.of(m_leftEncoder.getVelocity() / 60.0)
                 );
            },

            this
        )
    );

  private double m_prevLeftDist = 0;
  private double m_prevRightDist = 0;
  private double m_totalLeftDist = 0;
  private double m_totalRightDist = 0;

  private final SimpleMotorFeedforward m_lowGearFF = new SimpleMotorFeedforward(0.094402, 0.12423, 0.026461);
  private final SimpleMotorFeedforward m_highGearFF = new SimpleMotorFeedforward(0.05, 0.08, 0.015); // ! Change these

  private final AutoFactory autoFactory;

  /**
   * The main class to drive the robot. Used in {@link frc.robot.RobotContainer RobotContainer}.
   *
   * @param gyro This is a parameter that initializes the gyroscope for DriveSubsystem based on the RobotContainer
   * @param pneumatics This is a parameter that takes in the pneumatics from RobotContainer
  */

  // DONE: Add Odometry
  // DONE: SysID
  // DONE: Add Current Limits

  // TESTING: FeedForward
  // ? What is feedfoward? It makes the Choreo trajectory slightly more accurate based on SysID
  // TESTING: Delta Odometry
  // ? What does delta odometry? It prevents odometry jumping with gear switches
  // TESTING: Odometry logging in AdvantageScope
  // TESTING: PathPlanner

  // TODO: Make PhotonVision
  // TODO: Add PhotonVision info to logs

  public DriveSubsystem(Gyroscope gyro, Pneumatics pneumatics) {
    this.m_gyroscope = gyro;
    this.m_pneumatics = pneumatics;

    autoFactory = new AutoFactory(
      this::getPose, // A function that returns the current robot pose
      this::resetOdometry, // A function that resets the current robot pose to the provided Pose2d
      this::followTrajectory, // The drive subsystem trajectory follower
      true, // If alliance flipping should be enabled
      this // The drive subsystem
    );

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

    leftLeaderConfig.smartCurrentLimit(40);
    leftConfig.smartCurrentLimit(40);
    rightLeaderConfig.smartCurrentLimit(40);
    rightConfig.smartCurrentLimit(40);

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

    Logger.recordOutput("Robot/Pose", m_driveSim.getPose());
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
   * Converts raw motor RPM to Meters Per Second based on current gear.
   */
  private double getMetersPerSecond() {
      double currentRatio = m_pneumatics.isHighGear() ? highGearRatio : lowGearRatio;
      // 6 inch diameter, convert RPM to Rotations Per Second (/60)
      double unitConversion = (Units.inchesToMeters(6) * Math.PI) / (currentRatio * 60.0);

      double avgMotorRPM = (m_leftEncoder.getVelocity() + m_rightEncoder.getVelocity()) / 2.0;
      return Math.abs(avgMotorRPM * unitConversion);
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
    if (!isAutoShiftEnabled || autoShiftTimer > 0) {
      if (autoShiftTimer > 0) {
        autoShiftTimer--;
      }
      return;
    }

    double avgVelocity = getMetersPerSecond();
    boolean currentlyHigh = m_pneumatics.isHighGear();

    if (!currentlyHigh && avgVelocity > highGearThreshold) {
        m_pneumatics.setHighGear(true);
        autoShiftTimer = 15;
        System.out.println("Shifted HIGH. Speed was: " + avgVelocity);
    } 
    else if (currentlyHigh && avgVelocity < lowGearThreshold) {
        m_pneumatics.setHighGear(false);
        autoShiftTimer = 15;
        System.out.println("Shifted LOW. Speed was: " + avgVelocity);
    }
}

  @Override
  public void periodic() {
    double currentRatio = m_pneumatics.isHighGear() ? highGearRatio : lowGearRatio;
    double positionFactor = (Units.inchesToMeters(6) * Math.PI) / currentRatio;

    double currentLeftRaw = m_leftEncoder.getPosition();
    double currentRightRaw = m_rightEncoder.getPosition();

    double deltaLeftRotations = currentLeftRaw - m_prevLeftDist;
    double deltaRightRotations = currentRightRaw - m_prevRightDist;

    m_totalLeftDist += deltaLeftRotations * positionFactor;
    m_totalRightDist += deltaRightRotations * positionFactor;

    m_odometry.update(
        Rotation2d.fromDegrees(m_gyroscope.getHeading()),
        m_totalLeftDist,
        m_totalRightDist
    );

    m_prevLeftDist = currentLeftRaw;
    m_prevRightDist = currentRightRaw;

    autoShift();

    Logger.recordOutput("Robot/LeftEncoder", m_leftEncoder.getPosition());

    var currentPose = m_odometry.getPoseMeters();
    Logger.recordOutput("Robot/Pose", currentPose);
    Logger.recordOutput("Robot/Pose3d", new edu.wpi.first.math.geometry.Pose3d(currentPose));

    m_Field2d.setRobotPose(currentPose);
  }

  /**
   * Sets the voltages of the motors
   * @param leftVolts Amount of volts to apply to the left side
   * @param rightVolts Amount of volts to apply to the right side
   */
  public void setDriveVoltages(double leftVolts, double rightVolts) {
    m_leftLeader.setVoltage(leftVolts);
    m_rightLeader.setVoltage(rightVolts);

    m_drive.feed();
  }

  /**
   * Gets the pose of the robot from the odometry
   * @return The pose of the robot as a Pose2D
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * <b>Choreo Drive Chassis Speeds</b> <br>
   * Converts high-level robot movement (forward/turn) into specific wheel speeds. Used by Choreo
   * @param speeds a linear component that allows to get velocity
  */
  public void choreoDriveCS(ChassisSpeeds speeds) {
    DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(speeds);

    choreoDriveWV(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
  }

  /**
   * <b>Choreo Drive Wheel Velocities</b> <br>
   * Commands the motors to reach specific velocities in meters per second.
   * @param leftMpS Amount of MpS to apply to the left
   * @param rightMpS Amount of MpS to apply to the right
  */
  public void choreoDriveWV(double leftMpS, double rightMpS) {
    double currentRatio = m_pneumatics.isHighGear() ? highGearRatio : lowGearRatio;
    double wheelCircumference = Units.inchesToMeters(6) * Math.PI;

    double leftMotorRPS = (leftMpS / wheelCircumference) * currentRatio;
    double rightMotorRPS = (rightMpS / wheelCircumference) * currentRatio;

    var currentFF = getFeedforward();

    double leftVoltage = currentFF.calculate(leftMotorRPS);
    double rightVoltage = currentFF.calculate(rightMotorRPS);

    m_leftLeader.setVoltage(leftVoltage);
    m_rightLeader.setVoltage(rightVoltage);

    m_drive.feed();
  }

  /**
   *  Follows a Choreo trajectory
   * @param sample a Differential Sample
   */
  public void followTrajectory(DifferentialSample sample) {
    Pose2d pose = getPose();

    ChassisSpeeds ff = sample.getChassisSpeeds();

    ChassisSpeeds speeds = controller.calculate(
      pose,
      sample.getPose(),
      ff.vxMetersPerSecond,
      ff.omegaRadiansPerSecond
    );

    choreoDriveCS(speeds);
  }

  /**
   * The command to run a SysID Quasistatic
   * @param direction robot direction
   * @return the command to run a quasistatic SysID
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  /**
   * The command to run a SysID Dynamic
   * @param direction robot direction
   * @return the command to run a dynamic SysID
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  /**
   * Resets the odometry of the robot
   * @param pose Pose of the robot
   */
  public void resetOdometry(Pose2d pose) {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
    m_prevLeftDist = 0;
    m_prevRightDist = 0;
    m_totalLeftDist = 0;
    m_totalRightDist = 0;
    m_odometry.resetPosition(Rotation2d.fromDegrees(m_gyroscope.getHeading()), 0, 0, pose);
  }

  /**
   * Get the gear's Feed Forward
   * @return The feed forward for the current gear
   */
  private SimpleMotorFeedforward getFeedforward() {
    return m_pneumatics.isHighGear() ? m_highGearFF : m_lowGearFF;
  }

  public Command testChoreo(){
    return Commands.sequence(
      autoFactory.resetOdometry("NewPath"),
      autoFactory.trajectoryCmd("NewPath")
    );
  }
}