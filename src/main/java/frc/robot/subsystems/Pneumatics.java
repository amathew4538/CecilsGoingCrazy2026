package frc.robot.subsystems;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase{
    private final Compressor m_compressor = new Compressor(5, PneumaticsModuleType.REVPH);
    private final DoubleSolenoid m_doubleSolenoidLeft = new DoubleSolenoid(5, PneumaticsModuleType.REVPH, 0, 1);
    private final DoubleSolenoid m_doubleSolenoidRight = new DoubleSolenoid(5, PneumaticsModuleType.REVPH, 14, 15);

    public Pneumatics(){
        m_compressor.enableDigital();

        m_doubleSolenoidLeft.set(kReverse);
        m_doubleSolenoidRight.set(kReverse);
    };

    public Command toggleSolenoids() {
        return this.runOnce(() -> {
            m_doubleSolenoidLeft.toggle();
            m_doubleSolenoidRight.toggle();
        });
    }
}
