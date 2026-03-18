// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.AutoShift;
import frc.robot.subsystems.ChoreoCommands;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Gyroscope;
import frc.robot.subsystems.OdometryManager;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.SysID;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Pneumatics m_pneumatics = new Pneumatics();
  private final Gyroscope m_gyroscope = new Gyroscope();
  private final AutoShift m_autoShift = new AutoShift(m_pneumatics);
  private final OdometryManager m_odometry = new OdometryManager(m_gyroscope);
    private final Vision m_vision = new Vision(m_odometry);
  private final DriveSubsystem m_drive = new DriveSubsystem(m_gyroscope, m_pneumatics, m_autoShift);
  @SuppressWarnings("unused")
  private final RobotInfo m_robotInfo = new RobotInfo(m_drive, m_gyroscope, m_pneumatics, m_autoShift, m_odometry, m_vision);
  private final ChoreoCommands m_choreo = new ChoreoCommands(m_drive, m_pneumatics, m_odometry);
  private final SysID m_sysID = new SysID(m_drive);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    updateControllerBindings();
  }

  /**
   * Use this method to define your trigger -> command mappings.
   *
   * <br>
   * <br>
   *
   * Triggers can be created via:
   * <ul>
   * <li>{@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate </li>
   * <li>{@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}</li>
   * <li>{@link CommandXboxController Xbox}
   * <li>{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}</li>
   * <li>{@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}</li>
   * </ul>
  */

  // In RobotContainer.java
  public void updateControllerBindings() {
    // Clear existing buttons if necessary, though this is tricky with Triggers
    if (DriverStation.getJoystickIsXbox(0)) {
        configureXboxBindings();
    } else {
        configurePS4Bindings();
    }
  }

  /**
   * Configure the bindings for Xbox controllers
   * @apiNote It's run by {@link #updateControllerBindings()}
  */
  private void configureXboxBindings() {
    final CommandXboxController m_XboxController = new CommandXboxController(0);

    if (Robot.isSimulation()){
      m_drive.setDefaultCommand(
          m_drive.run(() -> m_drive.arcadeDrive(
              -m_XboxController.getLeftY(),
              -m_XboxController.getRightX() * 0.5
          ))
      );
    }
    else {
      m_drive.setDefaultCommand(
          m_drive.run(() -> m_drive.arcadeDrive(
              -m_XboxController.getLeftY(),
              m_XboxController.getRightX()
          ))
      );
    }
    m_XboxController.rightBumper().onTrue(m_pneumatics.toggleSolenoids());

    m_XboxController.a().onTrue(m_drive.turn180());

    m_XboxController.y().onTrue(
      m_gyroscope.resetHeading()
      .alongWith(m_drive.resetSimPose())
    );

    m_XboxController.povUp().whileTrue(m_sysID.sysIdQuasistatic(SysIdRoutine.Direction.kForward));

    m_XboxController.povDown().whileTrue(m_sysID.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

    m_XboxController.povLeft().whileTrue(m_sysID.sysIdDynamic(SysIdRoutine.Direction.kForward));

    m_XboxController.povRight().whileTrue(m_sysID.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    m_XboxController.b().onTrue(m_choreo.testChoreo());
  }

  /**
   * Configure the bindings for PS4 controllers
   * @apiNote It's run by {@link #updateControllerBindings()}
  */
  private void configurePS4Bindings() {
    final CommandPS4Controller m_PS4Controller = new CommandPS4Controller(0);

    if (Robot.isSimulation()){
      m_drive.setDefaultCommand(
          m_drive.run(() -> m_drive.arcadeDrive(
              -m_PS4Controller.getLeftY(),
              -m_PS4Controller.getRightX() * 0.5
          ))
      );
    }

    else {
      m_drive.setDefaultCommand(
          m_drive.run(() -> m_drive.arcadeDrive(
              -m_PS4Controller.getLeftY(),
              -m_PS4Controller.getRightX()
          ))
      );
    }

    m_PS4Controller.R1().onTrue(m_pneumatics.toggleSolenoids());

    m_PS4Controller.triangle().onTrue(
      m_gyroscope.resetHeading()
      .alongWith(m_drive.resetSimPose())
    );

    m_PS4Controller.cross().onTrue(m_drive.turn180());

    m_PS4Controller.povUp().whileTrue(m_sysID.sysIdQuasistatic(SysIdRoutine.Direction.kForward));

    m_PS4Controller.povDown().whileTrue(m_sysID.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

    m_PS4Controller.povLeft().whileTrue(m_sysID.sysIdDynamic(SysIdRoutine.Direction.kForward));

    m_PS4Controller.povRight().whileTrue(m_sysID.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    m_PS4Controller.circle().onTrue(m_choreo.testChoreo());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }

  public DriveSubsystem getDrive() {
        return m_drive;
    }

    public Vision getVision() {
        return m_vision;
    }
}
