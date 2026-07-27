package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.Turn180;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode; //C: this is mass importation

public class DriveSubsystem extends SubsystemBase {
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

  private final Gyroscope m_gyro;
  private final Pneumatics m_pneumatics;
  private final AutoShift m_autoShift;

  /**
   * The main class to drive the robot. Used in {@link frc.robot.RobotContainer RobotContainer}.
   *
   * @param gyro This is a parameter that initializes the gyroscope for DriveSubsystem based on the RobotContainer
   * @param pneumatics This is a parameter that takes in the pneumatics from RobotContainer
  */


  // DONE: Automatic Gear Switching
  // DONE: Add Odometry
  // DONE: SysID
  // DONE: Add Current Limits
  // DONE: Delta Odometry
  // ? What does delta odometry do? It prevents odometry jumping with gear switches
  // DONE: Create Constants In Seperate File
  // DONE: PhotonVision Sim
  // DONE: Spliting Methods Across Files
  // DONE: Make PhotonVision
  // DONE: Add PhotonVision info to logs


  // TESTING: FeedForward
  // ? What is feedfoward? It makes the Choreo trajectory slightly more accurate based on SysID
  // TESTING: PathPlanner

  public DriveSubsystem(Gyroscope gyro, Pneumatics pneumatics, AutoShift autoShift) {
    m_drive.setSafetyEnabled(false);

    this.m_gyro = gyro;
    this.m_pneumatics = pneumatics;
    this.m_autoShift = autoShift;

    DriveConstants.m_leftEncoder.setPosition(0);
    DriveConstants.m_rightEncoder.setPosition(0);

    DriveConstants.m_leftLeader.configure(DriveConfigs.getConfig(false), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    DriveConstants.m_leftFollower.configure(DriveConfigs.getConfig(false, 4), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    DriveConstants.m_rightLeader.configure(DriveConfigs.getConfig(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    DriveConstants.m_rightFollower.configure(DriveConfigs.getConfig(true, 2), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


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
    return new Turn180(this, m_gyro);
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
   *
   * @return the MpS of the robot
  */
  public double getMetersPerSecond() {
    double currentRatio = m_pneumatics.isHighGear() ? DriveConstants.highGearRatio : DriveConstants.lowGearRatio;
    // 6 inch diameter, convert RPM to Rotations Per Second (/60)
    double unitConversion = (Units.inchesToMeters(6) * Math.PI) / (currentRatio * 60.0);

    double avgMotorRPM = (DriveConstants.m_leftEncoder.getVelocity() + DriveConstants.m_rightEncoder.getVelocity()) / 2.0;
    return Math.abs(avgMotorRPM * unitConversion);
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
   * Get the gear's Feed Forward
   * @return The feed forward for the current gear
   */
  @SuppressWarnings("unused")
  private SimpleMotorFeedforward getFeedforward() {
    return m_pneumatics.isHighGear() ? DriveConstants.m_highGearFF : DriveConstants.m_lowGearFF;
  }

  /**
   * Gets the sim pose
   *
   * @return a Pose2D for the sim pose
   */
  public Pose2d getSimPose() {
    return m_driveSim.getPose();
  }

  /**
   * Allows running the feed command
   */
  public void doFeed() {
    m_drive.feed();
  }

  @Override
  public void simulationPeriodic() {
    double currentRatio = m_pneumatics.isHighGear() ? DriveConstants.highGearRatio : DriveConstants.lowGearRatio;

    m_driveSim.setCurrentGearing(currentRatio);

    double leftVolts = DriveConstants.m_leftLeader.get() * 12.0;
    double rightVolts = DriveConstants.m_rightLeader.get() * 12.0;

    if (Math.abs(leftVolts) < 0.01) leftVolts = DriveConstants.m_leftLeader.getAppliedOutput() * 12.0;
    if (Math.abs(rightVolts) < 0.01) rightVolts = DriveConstants.m_rightLeader.getAppliedOutput() * 12.0;

    m_driveSim.setInputs(leftVolts, rightVolts);
    m_driveSim.update(0.020);

    DriveConstants.m_leftEncoder.setPosition(m_driveSim.getLeftPositionMeters());
    DriveConstants.m_rightEncoder.setPosition(m_driveSim.getRightPositionMeters());
    m_gyro.setSimHeading(m_driveSim.getHeading().getDegrees());
  }

  @Override
  public void periodic() {
    double currentRatio = m_pneumatics.isHighGear() ? DriveConstants.highGearRatio : DriveConstants.lowGearRatio;
    double positionFactor = ((Units.inchesToMeters(6) * Math.PI) / currentRatio) * DriveConstants.wheelCoefficient;

    double currentLeftRaw = DriveConstants.m_leftEncoder.getPosition();
    double currentRightRaw = DriveConstants.m_rightEncoder.getPosition();

    double deltaLeftRotations = currentLeftRaw - DriveConstants.m_prevLeftDist;
    double deltaRightRotations = currentRightRaw - DriveConstants.m_prevRightDist;

    DriveConstants.m_totalLeftDist += deltaLeftRotations * positionFactor;
    DriveConstants.m_totalRightDist += deltaRightRotations * positionFactor;

    DriveConstants.m_prevLeftDist = currentLeftRaw;
    DriveConstants.m_prevRightDist = currentRightRaw;

    m_autoShift.runAutoShift(this.getMetersPerSecond());
    // m_pneumatics.setHighGear(true);

    if (m_pneumatics.isHighGear()){
      DriveConstants.currentlyHighLogger = true;
    }
    else if (!m_pneumatics.isHighGear()) {
      DriveConstants.currentlyHighLogger = false;
    }
  }
}