package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Pneumatics extends SubsystemBase {
    private final PneumaticHub m_pneumaticHub = new PneumaticHub(5);

    private final DoubleSolenoid m_solenoidLeft = m_pneumaticHub.makeDoubleSolenoid(0, 1);
    private final DoubleSolenoid m_solenoidRight = m_pneumaticHub.makeDoubleSolenoid(14, 15);

    private final ShuffleboardTab m_tab = Shuffleboard.getTab("Main");
    
    private final GenericEntry m_compressorEntry = m_tab.add("Compressor On", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .getEntry();

    private final GenericEntry m_pressureFullEntry = m_tab.add("Pressure Full", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .getEntry();

    public Pneumatics() {
        m_pneumaticHub.enableCompressorDigital();

        m_solenoidLeft.set(kReverse);
        m_solenoidRight.set(kReverse);
    }

    @Override
    public void periodic() {
        m_compressorEntry.setBoolean(m_pneumaticHub.getCompressor());
        m_pressureFullEntry.setBoolean(!m_pneumaticHub.getPressureSwitch());
    }

    public Command toggleSolenoids() {
        return this.runOnce(() -> {
            Value targetValue = (m_solenoidLeft.get() == kForward) ? kReverse : kForward;
            m_solenoidLeft.set(targetValue);
            m_solenoidRight.set(targetValue);
         });
    }
}
