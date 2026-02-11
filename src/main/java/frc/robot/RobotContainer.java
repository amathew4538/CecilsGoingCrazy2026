// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Gyroscope;
import frc.robot.subsystems.Pneumatics;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();


  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final CommandXboxController m_xbox = new CommandXboxController(0);
  private final CommandPS4Controller m_ps4 = new CommandPS4Controller(0);
  private final Pneumatics m_pneumatics = new Pneumatics();
  private final Gyroscope m_Gyroscope = new Gyroscope();


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
    m_robotDrive.setDefaultCommand(
        m_robotDrive.run(() -> {
            boolean isPS = m_ps4.getHID().getName().contains("Wireless");
            
            double speed = -m_xbox.getLeftY(); // Axis 1 is same for both
            double rotation = isPS ? -m_ps4.getRightX() : -m_xbox.getRightX();

            m_robotDrive.arcadeDrive(speed, rotation);
        })
    );

    getTopButton().onTrue(m_Gyroscope.resetHeading());
    getRightBumper().onTrue(m_pneumatics.toggleSolenoids());

    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    getSideButton().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    getRightBumper().onTrue(m_pneumatics.toggleSolenoids());
    getTopButton().onTrue(m_Gyroscope.resetHeading());
  }
  private Trigger getTopButton() {
    return m_ps4.getHID().getName().contains("Wireless") 
        ? m_ps4.triangle() 
        : m_xbox.y();
  }

  /** Helper to find the "Right Bumper" regardless of controller type */
  private Trigger getRightBumper() {
    return m_ps4.getHID().getName().contains("Wireless") 
        ? m_ps4.R1() 
        : m_xbox.rightBumper();
  }

  private Trigger getSideButton() {
    return m_ps4.getHID().getName().contains("Wireless")
      ? m_ps4.circle()
      : m_xbox.b();
}
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
