package frc.robot.subsystems;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase {
    private final PneumaticHub m_pneumaticHub = new PneumaticHub(5);

    private final DoubleSolenoid m_solenoidLeft = m_pneumaticHub.makeDoubleSolenoid(0, 1);
    private final DoubleSolenoid m_solenoidRight = m_pneumaticHub.makeDoubleSolenoid(14, 15);

    public Pneumatics() {
        m_pneumaticHub.enableCompressorDigital();

        m_solenoidLeft.set(kReverse);
        m_solenoidRight.set(kReverse);
    }

    public Command toggleSolenoids() {
        return this.runOnce(() -> {
            m_solenoidLeft.toggle();
            m_solenoidRight.toggle();
        });
    }
}
