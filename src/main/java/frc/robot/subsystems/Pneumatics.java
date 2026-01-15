package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;

public class Pneumatics extends SubsystemBase {
    private final PneumaticHub m_pneumaticHub = new PneumaticHub(5);
    
    private final DoubleSolenoid m_solenoidLeft = m_pneumaticHub.makeDoubleSolenoid(0, 1);
    private final DoubleSolenoid m_solenoidRight = m_pneumaticHub.makeDoubleSolenoid(14, 15);

    public Pneumatics() {
        m_pneumaticHub.enableCompressorDigital();
        
        m_solenoidLeft.set(kReverse);
        m_solenoidRight.set(kReverse);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Compressor On", m_pneumaticHub.getCompressor());
        SmartDashboard.putBoolean("Pressure Switch", m_pneumaticHub.getPressureSwitch());
    }

    public Command toggleSolenoids() {
        if (m_solenoidLeft.get() == kReverse)
            return this.runOnce(() -> {
                m_solenoidLeft.set(kForward);
                m_solenoidRight.set(kForward);
            });
        else
            return this.runOnce(() -> {
                m_solenoidLeft.set(kReverse);
                m_solenoidRight.set(kReverse);
            });
    }
}