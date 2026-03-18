package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class AutoShift extends SubsystemBase{
    private Pneumatics m_pneumatics;

    public AutoShift(Pneumatics pneumatics) {
        this.m_pneumatics = pneumatics;
    }


    /**
     * A command to turn on and off auto shift
     *
     * @see #autoShift()
     *
     * @return A command that swaps {@link #isAutoShiftEnabled} and logs it
    */
    public Command toggleAutoShift() {
        return runOnce(() -> {
            DriveConstants.isAutoShiftEnabled = !DriveConstants.isAutoShiftEnabled;
        });
    }

    /**
     * Finds the average Meters Per Second of both Spark Maxes then shifts.
     *
     * <ul>
     * <li> <b>Reasons for shifting:</b> </li>
     * <ul>
     * <li> Shift up if speed hits {@link #highGearThreshold} </li>
     * <li> Shift down if speed hits {@link #lowGearThreshold} </li>
     * <li> Shift down if power draw is higher than {@link #highCurrentThreshold}
     * </ul>
     * </ul>
     *
     * @see Pneumatics#toggleSolenoids()
    */
    public void runAutoShift(double avgVelocity) {
        if (!DriveConstants.isAutoShiftEnabled || DriveConstants.autoShiftTimer > 0) {
          if (DriveConstants.autoShiftTimer > 0) {
            DriveConstants.autoShiftTimer--;
          }
          return;
        }

        boolean currentlyHigh = m_pneumatics.isHighGear();

        if (!currentlyHigh && avgVelocity > DriveConstants.highGearThreshold) {
            m_pneumatics.setHighGear(true);
            DriveConstants.autoShiftTimer = 15;
            System.out.println("Shifted HIGH. Speed was: " + avgVelocity);
        }
        else if (currentlyHigh && avgVelocity < DriveConstants.lowGearThreshold) {
            m_pneumatics.setHighGear(false);
            DriveConstants.autoShiftTimer = 15;
            System.out.println("Shifted LOW. Speed was: " + avgVelocity);
        }
    }
}
