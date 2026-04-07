package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Pneumatics extends SubsystemBase {
    private final PneumaticHub m_pneumaticHub = new PneumaticHub(5);

    private final DoubleSolenoid m_solenoidLeft = m_pneumaticHub.makeDoubleSolenoid(0, 1);
    private final DoubleSolenoid m_solenoidRight = m_pneumaticHub.makeDoubleSolenoid(2, 6);

    /**
     * The main class used in the Pneumatics. Used in {@link frc.robot.RobotContainer RobotContainer}.
    */

    public Pneumatics() {
        m_pneumaticHub.enableCompressorDigital();

        m_solenoidLeft.set(kReverse);
        m_solenoidRight.set(kReverse);
    }

    /**
     * Switches the solenoids between states. For gear switching on the robot
     *
     * @return A command that switches the state of the solenoids
     */
    public Command toggleSolenoids() {
        return this.runOnce(() -> {
            Value targetValue = (m_solenoidLeft.get() == kForward) ? kReverse : kForward;
            m_solenoidLeft.set(targetValue);
            m_solenoidRight.set(targetValue);
         });
    }

    /**
     * Sets the drivetrain to a specific gear.
     *
     * @param highGear True for High Gear, false for Low Gear.
     *
     * @apiNote This is used by the {@link DriveSubsystem} for automatic shifting.
    */
    public void setHighGear(boolean highGear) {
        Value target = highGear ? kForward : kReverse;
        m_solenoidLeft.set(target);
        m_solenoidRight.set(target);
    }

    /**
     * Returns whether the robot is currently in high gear.
     *
     * @return True if the robot is in high gear
    */
    public boolean isHighGear() {
        return m_solenoidLeft.get() == kForward;
    }

    /**
     * Is the compressor running?
     *
     * @return boolean, whether the compressor is running
     */
    public boolean isCompressorRunning() {
        return m_pneumaticHub.getCompressor();
    }

    /**
     * Is the pressure full?
     *
     * @return boolean, whether the pressure is full
     */
    public boolean isPressureFull() {
       return !m_pneumaticHub.getPressureSwitch();
    }
}
