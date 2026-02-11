package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftLeader, m_rightLeader);

  public DriveSubsystem() {
    SparkMaxConfig leftConfig = new SparkMaxConfig(); //C: preparing to pair the left side
    SparkMaxConfig rightConfig = new SparkMaxConfig(); //C: ditto for right

    // Configure followers to follow their respective leaders
    leftConfig.follow(4); //C: now the code recognizes CanID 4 (the m_leftLeader) as a leader
    rightConfig.follow(2); //C: same with the right, numbers respective

    // Invert one side so positive power moves the robot forward
    rightConfig.inverted(true); //C: not much i can explain here when the above comment said it all already

    // 3. Apply configurations
    m_leftLeader.configure(new SparkMaxConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_leftFollower.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightLeader.configure(new SparkMaxConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightFollower.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    ShuffleboardTab m_tab = Shuffleboard.getTab("Drive System");

    m_tab.add("Drive Train", m_drive)
      .withWidget(BuiltInWidgets.kDifferentialDrive)
      .withSize(4, 3)
      .withPosition(0, 0);
  }

  public void arcadeDrive(double speed, double rotation) {
    m_drive.arcadeDrive(speed, rotation);
  }
}