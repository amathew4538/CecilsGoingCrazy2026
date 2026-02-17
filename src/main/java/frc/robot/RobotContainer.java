// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


// import frc.robot.commands.Autos;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Gyroscope;
import frc.robot.subsystems.Pneumatics;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
  private final DriveSubsystem m_robotDrive = new DriveSubsystem(m_gyroscope);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */

  private void configureBindings() {
    if (DriverStation.getJoystickIsXbox(0)) {
        configureXboxBindings();
    } else {
        configurePS4Bindings();
    }
  }

  private void configureXboxBindings() {
    final CommandXboxController m_XboxController = new CommandXboxController(0);

    if (Robot.isSimulation()){
      m_robotDrive.setDefaultCommand(
          m_robotDrive.run(() -> m_robotDrive.arcadeDrive(
              -m_XboxController.getLeftY(),
              -m_XboxController.getRightX() * 0.5
          ))
      );
    }
    else {
      m_robotDrive.setDefaultCommand(
          m_robotDrive.run(() -> m_robotDrive.arcadeDrive(
              -m_XboxController.getLeftY(),
              -m_XboxController.getRightX()
          ))
      );
    }
    m_XboxController.rightBumper().onTrue(m_pneumatics.toggleSolenoids());

    m_XboxController.a().onTrue(m_robotDrive.turn180());

    m_XboxController.y().onTrue(
      m_gyroscope.resetHeading()
      .alongWith(m_robotDrive.resetSimPose())
    );
  }

  private void configurePS4Bindings() {
    final CommandPS4Controller m_PS4Controller = new CommandPS4Controller(0);

    if (Robot.isSimulation()){
      m_robotDrive.setDefaultCommand(
          m_robotDrive.run(() -> m_robotDrive.arcadeDrive(
              -m_PS4Controller.getLeftY(),
              -m_PS4Controller.getRightX() * 0.5
          ))
      );
    }

    else {
      m_robotDrive.setDefaultCommand(
          m_robotDrive.run(() -> m_robotDrive.arcadeDrive(
              -m_PS4Controller.getLeftY(),
              -m_PS4Controller.getRightX()
          ))
      );
    }

    m_PS4Controller.R1().onTrue(m_pneumatics.toggleSolenoids());

    m_PS4Controller.triangle().onTrue(
      m_gyroscope.resetHeading()
      .alongWith(m_robotDrive.resetSimPose())
    );

    m_PS4Controller.cross().onTrue(m_robotDrive.turn180());
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
}
