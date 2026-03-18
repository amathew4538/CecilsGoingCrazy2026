package frc.robot.subsystems;

import choreo.auto.AutoFactory;
import choreo.trajectory.DifferentialSample;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.DriveConstants;

public class ChoreoCommands {
  private final DriveSubsystem m_drive;
  private final Pneumatics m_pneumatics;
  private final OdometryManager m_odometry;

  private final AutoFactory autoFactory;

  public ChoreoCommands(DriveSubsystem drive, Pneumatics pneumatics, OdometryManager odometry) {
    this.m_drive = drive;
    this.m_pneumatics = pneumatics;
    this.m_odometry = odometry;

    autoFactory = new AutoFactory(
      m_odometry::getPose, // A function that returns the current robot pose
      m_odometry::resetOdometry, // A function that resets the current robot pose to the provided Pose2d
      this::followTrajectory, // The drive subsystem trajectory follower
      true, // If alliance flipping should be enabled
      m_drive // The drive subsystem
    );
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
    double currentRatio = m_pneumatics.isHighGear() ? DriveConstants.highGearRatio : DriveConstants.lowGearRatio;
    double wheelCircumference = Units.inchesToMeters(6) * Math.PI;

    double leftMotorRPS = (leftMpS / wheelCircumference) * currentRatio;
    double rightMotorRPS = (rightMpS / wheelCircumference) * currentRatio;

    // var currentFF = getFeedforward();
    var currentFF = DriveConstants.m_lowGearFF;

    double leftVoltage = currentFF.calculate(leftMotorRPS);
    double rightVoltage = currentFF.calculate(rightMotorRPS);

    DriveConstants.m_leftLeader.setVoltage(leftVoltage);
    DriveConstants.m_rightLeader.setVoltage(rightVoltage);

    m_drive.doFeed();
  }

  /**
   *  Follows a Choreo trajectory
   * @param sample a Differential Sample
  */
  public void followTrajectory(DifferentialSample sample) {
    DriveConstants.isAutoShiftEnabled = false;
    Pose2d pose = m_odometry.getPose();

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
   * A command to test choreo
   * @return a command that resets odometry then runs a choreo path
  */
  public Command testChoreo(){
    return Commands.sequence(
      autoFactory.resetOdometry("NewPath"),
      autoFactory.trajectoryCmd("NewPath")
    );
  }
}
