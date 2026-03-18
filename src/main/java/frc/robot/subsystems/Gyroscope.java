package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
public class Gyroscope extends SubsystemBase{
    private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();
    private ADXRS450_GyroSim m_gyroSim;


    /**
     * The main class to initiate the gyro. Used in {@link frc.robot.RobotContainer RobotContainer}.
    */

    // ! Do not remove or it will break the code!
    public Gyroscope() {
        m_gyro.calibrate();
        m_gyro.reset();


        if (Robot.isSimulation()) {
            m_gyroSim = new ADXRS450_GyroSim(m_gyro);
        }
    }

    /**
     * Resets the heading of the Gyro
     *
     * @return A command to reset the gyro heading
    */
    public Command resetHeading() {
       return runOnce(() -> {m_gyro.reset();});
    }

    /**
     * Set Sim heading to certain degree
     * @param degrees Inputs a heading in degrees to set the Sim heading to
     */
    public void setSimHeading(double degrees) {
        if (m_gyroSim != null) {
            m_gyroSim.setAngle(degrees);
        }
    }

    /**
     * Gets the robot's heading
     *
     * @return The current heading of the robot in degrees
     */
    public double getHeading() {
        return m_gyro.getAngle();
    }
}
