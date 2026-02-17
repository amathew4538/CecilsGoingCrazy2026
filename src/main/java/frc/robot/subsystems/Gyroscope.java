package frc.robot.subsystems;

import java.util.Map;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Gyroscope extends SubsystemBase{
    private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();
    private ADXRS450_GyroSim m_gyroSim;

    public Gyroscope() {
        m_gyro.calibrate();
        m_gyro.reset();

        ShuffleboardTab m_tab = Shuffleboard.getTab("Sensors");

        m_tab.add("Robot Heading", m_gyro)
            .withWidget(BuiltInWidgets.kGyro)
            .withProperties(Map.of("Starting angle", 0));

        m_tab.add("Reset Gyro", resetHeading())
            .withWidget(BuiltInWidgets.kCommand)
            .withPosition(5, 0)
            .withSize(2, 1);

        if (Robot.isSimulation()) {
            m_gyroSim = new ADXRS450_GyroSim(m_gyro);
        }
    }

    public Command resetHeading() {
       return runOnce(() -> {m_gyro.reset();});
    }
    
    public void setSimHeading(double degrees) {
        if (m_gyroSim != null) {
            m_gyroSim.setAngle(degrees);
        }
    }

    public double getHeading() {
        return m_gyro.getAngle();
    }
}
