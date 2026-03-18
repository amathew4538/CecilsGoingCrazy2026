// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
// ! Uncomment for log replay
// import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import java.io.File;
import java.util.Arrays;
import java.util.Comparator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Vision;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private final Vision m_vision;
  private final DriveSubsystem m_drive;

  boolean wasXbox;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our autonomous chooser on the dashboard.

    if (Robot.isReal()) {
        Logger.recordMetadata("RobotMode", "REAL");
        String logPath = "/u/logs";
        if (!new File(logPath).exists()) logPath = "/home/lvuser/logs";

        cleanOldLogs(logPath, 5);

        Logger.addDataReceiver(new WPILOGWriter(logPath)); // Log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    } else {
        // ! Uncomment for log replay
        // setUseTiming(false);
        setUseTiming(true);

        // ! Uncomment for log replay
        // String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
        // Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.recordMetadata("RobotMode", "SIMULATION");
        Logger.addDataReceiver(new NT4Publisher()); // Read replay log
        Logger.addDataReceiver(new WPILOGWriter("sim_logs/")); // Save outputs to a new log
    }

    Logger.recordMetadata("ProjectName", "CecilsGoingCrazy2026"); // Set a metadata value
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.

    m_robotContainer = new RobotContainer();
    m_drive = m_robotContainer.getDrive();
    m_vision = m_robotContainer.getVision();
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
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

    wasXbox = DriverStation.getJoystickIsXbox(0);
    m_robotContainer.updateControllerBindings();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    boolean isXbox = DriverStation.getJoystickIsXbox(0);

    if (isXbox != wasXbox) {
      m_robotContainer.updateControllerBindings();
      wasXbox = isXbox;
    }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    m_vision.updateSimPose(m_drive.getSimPose());
  }

  /**
   * Cleans the logs up to the last {@code maxFiles} created
   * @param path path to the logs
   * @param maxFiles how many files left
   */
  private void cleanOldLogs(String path, int maxFiles) {
    File dir = new File(path);
    if (!dir.exists() || !dir.isDirectory()) return;

    File[] files = dir.listFiles((d, name) -> name.endsWith(".wpilog"));
    if (files != null && files.length > maxFiles) {
        // Sort files by last modified date (oldest first)
        Arrays.sort(files, Comparator.comparingLong(File::lastModified));

        // Delete files from the start of the array until only maxFiles remain
        for (int i = 0; i < files.length - maxFiles; i++) {
            if (files[i].delete()) {
                System.out.println("Deleted old log: " + files[i].getName());
            }
        }
    }
}
}
