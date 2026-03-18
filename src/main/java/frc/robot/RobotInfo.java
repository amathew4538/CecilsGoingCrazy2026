package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.AutoShift;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Gyroscope;
import frc.robot.subsystems.OdometryManager;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Vision;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class RobotInfo extends SubsystemBase {
  private final DriveSubsystem m_drive;
  private final Gyroscope m_gyro;
  private final Pneumatics m_pneumatics;
  private final AutoShift m_autoShift;
  private final OdometryManager m_odometry;
  private final Vision m_vision;

  private GenericEntry m_compressor;
  private GenericEntry m_pressureFull;
  private GenericEntry m_gear;

  public RobotInfo(DriveSubsystem drive, Gyroscope gyro, Pneumatics pneumatics, AutoShift autoShift, OdometryManager odometry, Vision vision) {
    this.m_drive = drive;
    this.m_gyro = gyro;
    this.m_pneumatics = pneumatics;
    this.m_autoShift = autoShift;
    this.m_odometry = odometry;
    this.m_vision = vision;

    ShuffleboardTab m_driveTab = Shuffleboard.getTab("Drive System");
    ShuffleboardTab m_gyroTab = Shuffleboard.getTab("Sensors");
    ShuffleboardTab m_pneumaticsTab = Shuffleboard.getTab("Pneumatics");
    ShuffleboardTab m_visionTab = Shuffleboard.getTab("Vision");

    m_driveTab.add("Drive Train", m_drive)
      .withWidget(BuiltInWidgets.kDifferentialDrive)
      .withSize(4, 3)
      .withPosition(0, 0);

    m_driveTab.add("180 PID", DriveConstants.m_pid)
      .withWidget(BuiltInWidgets.kPIDCommand);

    m_driveTab.add("Field", DriveConstants.m_Field2d)
      .withWidget(BuiltInWidgets.kField)
      .withSize(4, 6)
      .withPosition(4, 0);

    m_driveTab.add("Toggle Auto Shift", m_autoShift.toggleAutoShift())
      .withWidget(BuiltInWidgets.kCommand)
      .withPosition(5, 0)
      .withSize(2, 1);

    m_driveTab.add("Auto Shift On", DriveConstants.isAutoShiftEnabled)
      .withWidget(BuiltInWidgets.kBooleanBox)
      .getEntry();

    m_gyroTab.add("Robot Heading", m_gyro)
      .withWidget(BuiltInWidgets.kGyro)
      .withProperties(Map.of("Starting angle", 0));

    m_gyroTab.add("Reset Gyro", m_gyro.resetHeading())
      .withWidget(BuiltInWidgets.kCommand)
      .withPosition(5, 0)
      .withSize(2, 1);

    m_compressor = m_pneumaticsTab.add("Compressor On", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .getEntry();

    m_pressureFull = m_pneumaticsTab.add("Pressure Full", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .getEntry();

    m_gear = m_pneumaticsTab.add("High Gear?", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .getEntry();

    m_gear.setBoolean(false);

    m_pneumaticsTab.add("Switch Gear", m_pneumatics.toggleSolenoids())
      .withWidget(BuiltInWidgets.kCommand)
      .withPosition(5, 0)
      .withSize(2, 1);

    m_visionTab.add("Has Target?", m_vision.hasTargets())
      .withWidget(BuiltInWidgets.kBooleanBox)
      .getEntry();
    }

  @Override
  public void periodic() {
    m_compressor.setBoolean(m_pneumatics.isCompressorRunning());
    m_pressureFull.setBoolean(!m_pneumatics.isPressureFull());
    double avgVelocity = m_drive.getMetersPerSecond();
    m_gear.setBoolean(m_pneumatics.isHighGear());

    if (Robot.isSimulation()) {
      Logger.recordOutput("Robot/Drive/Pose", m_drive.getSimPose());
      Logger.recordOutput("Robot/Drive/Pose3d", new edu.wpi.first.math.geometry.Pose3d(m_drive.getSimPose()));
    } else if (Robot.isReal()) {
      Logger.recordOutput("Robot/Drive/Pose", m_odometry.getPose());
      Logger.recordOutput("Robot/Drive/Pose3d", new edu.wpi.first.math.geometry.Pose3d(m_odometry.getPose()));
    }
    Logger.recordOutput("Robot/Drive/AverageVelocity", avgVelocity);

    Logger.recordOutput("Robot/Pneumatics/AutoShiftEnabled", DriveConstants.isAutoShiftEnabled);
    Logger.recordOutput("Robot/Pneumatics/IsHighGear", DriveConstants.currentlyHighLogger);
    Logger.recordOutput("Robot/Pneumatics/LeftEncoder", DriveConstants.m_leftEncoder.getPosition());

    Logger.recordOutput("Robot/Gyro/Heading", m_gyro.getHeading());

    Logger.recordOutput("Robot/Vision/HasTarget", m_vision.hasTargets());
  }
}
