package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

import com.revrobotics.PersistMode;
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

  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftLeader, m_rightLeader);

  private final PIDController m_pid = new PIDController(0.03, 0, 0.002);

  private final DifferentialDrivetrainSim m_driveSim = new DifferentialDrivetrainSim(
    DCMotor.getNEO(2),
    7.29,
    2.1,
    Units.lbsToKilograms(50),
    Units.inchesToMeters(3),
    0.7112,
    null
  );

  private final edu.wpi.first.wpilibj.smartdashboard.Field2d m_Field2d = new Field2d();

  public DriveSubsystem(Gyroscope gyro) {
    this.m_gyroscope = gyro;

    SparkMaxConfig leftConfig = new SparkMaxConfig(); //C: preparing to pair the left side
    SparkMaxConfig rightLeaderConfig = new SparkMaxConfig();
    SparkMaxConfig rightConfig = new SparkMaxConfig(); //C: ditto for right

    // Configure followers to follow their respective leaders
    leftConfig.follow(4); //C: now the code recognizes CanID 4 (the m_leftLeader) as a leader
    rightConfig.follow(2); //C: same with the right, numbers respective

    // Invert one side so positive power moves the robot forward
    rightConfig.inverted(true); //C: not much i can explain here when the above comment said it all already
    rightLeaderConfig.inverted(true);

    // 3. Apply configurations
    m_leftLeader.configure(new SparkMaxConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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

    m_pid.enableContinuousInput(-180, 180);
    m_pid.setTolerance(2);
  }

  public void arcadeDrive(double speed, double rotation) {
    m_drive.arcadeDrive(speed, rotation);
  }

public Command turn180() {
  return runOnce(() -> {
      double target = edu.wpi.first.math.MathUtil.inputModulus(m_gyroscope.getHeading() + 180, -180, 180);
      m_pid.setSetpoint(target);
  })
  .andThen(
    run(() -> {
      double rotationSpeed = m_pid.calculate(m_gyroscope.getHeading());
      rotationSpeed = Math.max(-0.5, Math.min(0.5, rotationSpeed));
      this.arcadeDrive(0, rotationSpeed);
    })
  )
  .until(m_pid::atSetpoint)
  .finallyDo((interrupted) -> this.arcadeDrive(0, 0));
}

  @Override
  public void simulationPeriodic() {
    // bypass SparkSim entirely by using .get()
    m_driveSim.setInputs(m_leftLeader.get() * 12.0, m_rightLeader.get() * 12.0);
    m_driveSim.update(0.020);
    m_gyroscope.setSimHeading(m_driveSim.getHeading().getDegrees());
    m_Field2d.setRobotPose(m_driveSim.getPose());
  }

  public Command resetSimPose() {
      return runOnce(() -> {
        if (Robot.isSimulation()) {
          m_driveSim.setPose(new edu.wpi.first.math.geometry.Pose2d());
        }
      });
  }
}