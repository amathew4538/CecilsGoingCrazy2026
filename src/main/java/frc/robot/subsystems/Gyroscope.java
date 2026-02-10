package frc.robot.subsystems;

import java.util.Map;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gyroscope extends SubsystemBase{
    ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

    private final ShuffleboardTab m_tab = Shuffleboard.getTab("Sensors");

    private final GenericEntry m_gyroEntry = m_tab.add("Robot Heading", 0)
        .withWidget(BuiltInWidgets.kGyro)
        .withProperties(Map.of("Starting Angle", 0))
        .getEntry();

    public Gyroscope() {
        m_gyro.calibrate();
        m_gyro.reset();
    }

    public void resetHeading() {
        m_gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(m_gyro.getAngle(), 360);
    }

    @Override
    public void periodic() {
        m_gyroEntry.setDouble(getHeading());
    }
}
