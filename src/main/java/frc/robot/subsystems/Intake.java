package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final Spark m_motor = new Spark(0); // remember to change that number

    public Intake() {}

    public Command toggleIntake() {
        return runOnce(() -> {
            m_motor.set(m_motor.get() != 0 ? 0.8 : 0);
        });
    }
    // public Command stopIntake() {
    //     return runOnce(() -> {m_motor.set(0);});
    // }

}
