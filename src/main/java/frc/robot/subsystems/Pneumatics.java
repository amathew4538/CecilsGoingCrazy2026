package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.DriveSubsystem;

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

    private final ShuffleboardTab m_tab = Shuffleboard.getTab("Pneumatics");

    private final GenericEntry m_compressor = m_tab.add("Compressor On", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .getEntry();

    private final GenericEntry m_pressureFull = m_tab.add("Pressure Full", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .getEntry();

    private final GenericEntry m_gear = m_tab.add("High gear?", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .getEntry();

    /**
     * The main class used in the Pneumatics. Used in {@link frc.robot.RobotContainer RobotContainer}.
    */

    // DONE: Automatic Gear Switching

    public Pneumatics() {
        m_pneumaticHub.enableCompressorDigital();

        m_solenoidLeft.set(kReverse);
        m_solenoidRight.set(kReverse);

        m_gear.setBoolean(false);

        m_tab.add("Switch Gear", toggleSolenoids())
            .withWidget(BuiltInWidgets.kCommand)
            .withPosition(5, 0)
            .withSize(2, 1);
    }

    @Override
    public void periodic() {
        m_compressor.setBoolean(m_pneumaticHub.getCompressor());
        m_pressureFull.setBoolean(!m_pneumaticHub.getPressureSwitch());

        m_gear.setBoolean(m_solenoidLeft.get() == kForward);
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
}
