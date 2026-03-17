package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Gyroscope;
import frc.robot.subsystems.Pneumatics;
import org.littletonrobotics.junction.Logger;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import choreo.auto.AutoFactory;
import choreo.trajectory.DifferentialSample; //C: this is mass importation

public class DriveSubsystem extends SubsystemBase {
  private final DifferentialDriveOdometry m_odometry;

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
                DriveConstants.m_leftLeader.setVoltage(voltage);
                DriveConstants.m_rightLeader.setVoltage(voltage);
                m_drive.feed();
            },

            (SysIdRoutineLog log) -> {
              log.motor("Drivetrain")
               .voltage(
                    edu.wpi.first.units.Units.Volts.of(
                        DriveConstants.m_leftLeader.getAppliedOutput()
                            * DriveConstants.m_leftLeader.getBusVoltage()
                    )
                )
                .angularPosition(
                  edu.wpi.first.units.Units.Rotations.of(DriveConstants.m_leftEncoder.getPosition())
                )
                .angularVelocity(
                  edu.wpi.first.units.Units.RotationsPerSecond.of(DriveConstants.m_leftEncoder.getVelocity() / 60.0)
                 );
            },

            this
        )
    );

  private final AutoFactory autoFactory;

  // These are here just to be safe
  public final static DifferentialDrive m_drive = new DifferentialDrive(DriveConstants.m_leftLeader, DriveConstants.m_rightLeader);

  // These are here just to be safe
  public final static DifferentialDrivetrainSim m_driveSim = new DifferentialDrivetrainSim(
    DCMotor.getNEO(2),
    5.0,
    4.22,
    Units.lbsToKilograms(60),
    Units.inchesToMeters(3),
    Units.inchesToMeters(25),
    null
  );

  /**
   * The main class to drive the robot. Used in {@link frc.robot.RobotContainer RobotContainer}.
   *
   * @param gyro This is a parameter that initializes the gyroscope for DriveSubsystem based on the RobotContainer
   * @param pneumatics This is a parameter that takes in the pneumatics from RobotContainer
  */

  // DONE: Add Odometry
  // DONE: SysID
  // DONE: Add Current Limits
  // DONE: Delta Odometry
  // ? What does delta odometry? It prevents odometry jumping with gear switches
  // DONE: Create Constants In Seperate File

  // TESTING: FeedForward
  // ? What is feedfoward? It makes the Choreo trajectory slightly more accurate based on SysID
  // TESTING: Odometry logging in AdvantageScope
  // TESTING: PathPlanner

  // TODO: Spliting Methods Across Files
  // TODO: Make PhotonVision
  // TODO: Add PhotonVision info to logs
  // TODO: Add PhotonVision to Choreo
  // TODO: PhotonVision Sim

  public DriveSubsystem(Gyroscope gyro, Pneumatics pneumatics) {
    DriveConstants.m_gyroscope = gyro;
    DriveConstants.m_pneumatics = pneumatics;

    autoFactory = new AutoFactory(
      this::getPose, // A function that returns the current robot pose
      this::resetOdometry, // A function that resets the current robot pose to the provided Pose2d
      this::followTrajectory, // The drive subsystem trajectory follower
      true, // If alliance flipping should be enabled
      this // The drive subsystem
    );

    DriveConstants.m_leftEncoder.setPosition(0);
    DriveConstants.m_rightEncoder.setPosition(0);

    this.m_odometry = new DifferentialDriveOdometry(
      Rotation2d.fromDegrees(DriveConstants.m_gyroscope.getHeading()),
      0.0,
      0.0
    );

    DriveConstants.m_leftLeader.configure(DriveConfigs.getConfig(false), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    DriveConstants.m_leftFollower.configure(DriveConfigs.getConfig(false, 4), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    DriveConstants.m_rightLeader.configure(DriveConfigs.getConfig(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    DriveConstants.m_rightFollower.configure(DriveConfigs.getConfig(true, 2), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    ShuffleboardTab m_tab = Shuffleboard.getTab("Drive System");

    m_tab.add("Drive Train", m_drive)
      .withWidget(BuiltInWidgets.kDifferentialDrive)
      .withSize(4, 3)
      .withPosition(0, 0);

    m_tab.add("180 PID", DriveConstants.m_pid)
      .withWidget(BuiltInWidgets.kPIDCommand);

    m_tab.add("Field", DriveConstants.m_Field2d)
      .withWidget(BuiltInWidgets.kField)
      .withSize(4, 6)
      .withPosition(4, 0);

    m_tab.add("Toggle Auto Shift", toggleAutoShift())
      .withWidget(BuiltInWidgets.kCommand)
      .withPosition(5, 0)
      .withSize(2, 1);

    m_tab.add("Auto Shift On", DriveConstants.isAutoShiftEnabled)
      .withWidget(BuiltInWidgets.kBooleanBox)
      .getEntry();

    DriveConstants.m_pid.enableContinuousInput(-180, 180);
    DriveConstants.m_pid.setTolerance(2);
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
          double target = MathUtil.inputModulus(DriveConstants.m_gyroscope.getHeading() + 180, -180, 180);
          DriveConstants.m_pid.setSetpoint(target);
        }
      ),
      run(
        () -> {
          double rotationSpeed = DriveConstants.m_pid.calculate(DriveConstants.m_gyroscope.getHeading());
          rotationSpeed = Math.max(-0.75, Math.min(0.75, rotationSpeed));
          this.arcadeDrive(0, rotationSpeed);
        }
      ).until(
        DriveConstants.m_pid::atSetpoint
      )
    ).finallyDo(
      () -> CommandScheduler.getInstance().cancel(this.turn180())
    );
  }

  @Override
  public void simulationPeriodic() {
    double currentRatio = DriveConstants.m_pneumatics.isHighGear() ? DriveConstants.highGearRatio : DriveConstants.lowGearRatio;

    m_driveSim.setCurrentGearing(currentRatio);

    m_driveSim.setInputs(DriveConstants.m_leftLeader.get() * 12.0, DriveConstants.m_rightLeader.get() * 12.0);
    m_driveSim.update(0.020);

    DriveConstants.m_leftEncoder.setPosition(m_driveSim.getLeftPositionMeters());
    DriveConstants.m_rightEncoder.setPosition(m_driveSim.getRightPositionMeters());
    DriveConstants.m_gyroscope.setSimHeading(m_driveSim.getHeading().getDegrees());

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
      double currentRatio = DriveConstants.m_pneumatics.isHighGear() ? DriveConstants.highGearRatio : DriveConstants.lowGearRatio;
      // 6 inch diameter, convert RPM to Rotations Per Second (/60)
      double unitConversion = (Units.inchesToMeters(6) * Math.PI) / (currentRatio * 60.0);

      double avgMotorRPM = (DriveConstants.m_leftEncoder.getVelocity() + DriveConstants.m_rightEncoder.getVelocity()) / 2.0;
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
        DriveConstants.isAutoShiftEnabled = !DriveConstants.isAutoShiftEnabled;
        Logger.recordOutput("Robot/AutoShiftEnabled", DriveConstants.isAutoShiftEnabled);
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
    if (!DriveConstants.isAutoShiftEnabled || DriveConstants.autoShiftTimer > 0) {
      if (DriveConstants.autoShiftTimer > 0) {
        DriveConstants.autoShiftTimer--;
      }
      return;
    }

    double avgVelocity = getMetersPerSecond();
    boolean currentlyHigh = DriveConstants.m_pneumatics.isHighGear();

    if (!currentlyHigh && avgVelocity > DriveConstants.highGearThreshold) {
        DriveConstants.m_pneumatics.setHighGear(true);
        DriveConstants.autoShiftTimer = 15;
        System.out.println("Shifted HIGH. Speed was: " + avgVelocity);
    } 
    else if (currentlyHigh && avgVelocity < DriveConstants.lowGearThreshold) {
        DriveConstants.m_pneumatics.setHighGear(false);
        DriveConstants.autoShiftTimer = 15;
        System.out.println("Shifted LOW. Speed was: " + avgVelocity);
    }
}

  @Override
  public void periodic() {
    double currentRatio = DriveConstants.m_pneumatics.isHighGear() ? DriveConstants.highGearRatio : DriveConstants.lowGearRatio;
    double wheelCoefficient = 1.0933;
    double positionFactor = ((Units.inchesToMeters(6) * Math.PI) / currentRatio) * wheelCoefficient;

    double currentLeftRaw = DriveConstants.m_leftEncoder.getPosition();
    double currentRightRaw = DriveConstants.m_rightEncoder.getPosition();

    double deltaLeftRotations = currentLeftRaw - DriveConstants.m_prevLeftDist;
    double deltaRightRotations = currentRightRaw - DriveConstants.m_prevRightDist;

    DriveConstants.m_totalLeftDist += deltaLeftRotations * positionFactor;
    DriveConstants.m_totalRightDist += deltaRightRotations * positionFactor;

    double avgVelocity = getMetersPerSecond();
    Logger.recordOutput("Robot/AverageVelocity", avgVelocity);

    m_odometry.update(
        Rotation2d.fromDegrees(DriveConstants.m_gyroscope.getHeading()),
        DriveConstants.m_totalLeftDist,
        DriveConstants.m_totalRightDist
    );

    DriveConstants.m_prevLeftDist = currentLeftRaw;
    DriveConstants.m_prevRightDist = currentRightRaw;

    autoShift();
    // m_pneumatics.setHighGear(true);

    // Logger.recordOutput("Robot/LeftEncoder", m_leftEncoder.getPosition());

    var currentPose = m_odometry.getPoseMeters();
    Logger.recordOutput("Robot/Pose", currentPose);
    Logger.recordOutput("Robot/Pose3d", new edu.wpi.first.math.geometry.Pose3d(currentPose));

    DriveConstants.m_Field2d.setRobotPose(currentPose);

    if (DriveConstants.m_pneumatics.isHighGear()){
      DriveConstants.currentlyHighLogger = true;
      Logger.recordOutput("Robot/IsHighGear", DriveConstants.currentlyHighLogger);
    }
    else if (!DriveConstants.m_pneumatics.isHighGear()) {
      DriveConstants.currentlyHighLogger = false;
      Logger.recordOutput("Robot/IsHighGear", DriveConstants.currentlyHighLogger);
    }
  }

  /**
   * Sets the voltages of the motors
   * @param leftVolts Amount of volts to apply to the left side
   * @param rightVolts Amount of volts to apply to the right side
   */
  public void setDriveVoltages(double leftVolts, double rightVolts) {
    DriveConstants.m_leftLeader.setVoltage(leftVolts);
    DriveConstants.m_rightLeader.setVoltage(rightVolts);

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
    DifferentialDriveWheelSpeeds wheelSpeeds = DriveConstants.m_kinematics.toWheelSpeeds(speeds);

    choreoDriveWV(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
  }

  /**
   * <b>Choreo Drive Wheel Velocities</b> <br>
   * Commands the motors to reach specific velocities in meters per second.
   * @param leftMpS Amount of MpS to apply to the left
   * @param rightMpS Amount of MpS to apply to the right
  */
  public void choreoDriveWV(double leftMpS, double rightMpS) {
    double currentRatio = DriveConstants.m_pneumatics.isHighGear() ? DriveConstants.highGearRatio : DriveConstants.lowGearRatio;
    double wheelCircumference = Units.inchesToMeters(6) * Math.PI;

    double leftMotorRPS = (leftMpS / wheelCircumference) * currentRatio;
    double rightMotorRPS = (rightMpS / wheelCircumference) * currentRatio;

    // var currentFF = getFeedforward();
    var currentFF = DriveConstants.m_lowGearFF;

    double leftVoltage = currentFF.calculate(leftMotorRPS);
    double rightVoltage = currentFF.calculate(rightMotorRPS);

    DriveConstants.m_leftLeader.setVoltage(leftVoltage);
    DriveConstants.m_rightLeader.setVoltage(rightVoltage);

    m_drive.feed();
  }

  /**
   *  Follows a Choreo trajectory
   * @param sample a Differential Sample
   */
  public void followTrajectory(DifferentialSample sample) {
    DriveConstants.isAutoShiftEnabled = false;
    Pose2d pose = getPose();

    ChassisSpeeds ff = sample.getChassisSpeeds();

    ChassisSpeeds speeds = DriveConstants.controller.calculate(
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
    DriveConstants.m_leftEncoder.setPosition(0);
    DriveConstants.m_rightEncoder.setPosition(0);

    DriveConstants.m_prevLeftDist = 0;
    DriveConstants.m_prevRightDist = 0;
    DriveConstants.m_totalLeftDist = 0;
    DriveConstants.m_totalRightDist = 0;

    m_odometry.resetPosition(Rotation2d.fromDegrees(DriveConstants.m_gyroscope.getHeading()), 0, 0, pose);
  }

  /**
   * Get the gear's Feed Forward
   * @return The feed forward for the current gear
   */
  // private SimpleMotorFeedforward getFeedforward() {
    // return m_pneumatics.isHighGear() ? m_highGearFF : m_lowGearFF;
  // }

  public Command testChoreo(){
    return Commands.sequence(
      autoFactory.resetOdometry("NewPath"),
      autoFactory.trajectoryCmd("NewPath")
    );
  }
}