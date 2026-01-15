package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
public class DriveSubsystem extends SubsystemBase {
  private final SparkMax m_leftLeader = new SparkMax(4, MotorType.kBrushless);
  private final SparkMax m_leftFollower = new SparkMax(3, MotorType.kBrushless);
  private final SparkMax m_rightLeader = new SparkMax(2, MotorType.kBrushless);
  private final SparkMax m_rightFollower = new SparkMax(1, MotorType.kBrushless);

  public DriveSubsystem() {
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    SparkMaxConfig rightConfig = new SparkMaxConfig();

    // Configure followers to follow their respective leaders
    leftConfig.follow(4);
    rightConfig.follow(2);
    
    // Invert one side so positive power moves the robot forward
    rightConfig.inverted(true);
    
    // 3. Apply configurations
    m_leftLeader.configure(new SparkMaxConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_leftFollower.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightLeader.configure(new SparkMaxConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightFollower.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
  
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftLeader, m_rightLeader);
  
  public void arcadeDrive(double speed, double rotation) {
    m_drive.arcadeDrive(speed, rotation);
  }
}