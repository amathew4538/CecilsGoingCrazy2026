package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriveConstants;

public class SysID {
    private final DriveSubsystem m_drive;
    private final SysIdRoutine m_sysIdRoutine;

    public SysID(DriveSubsystem drive) {
        this.m_drive = drive;

        this.m_sysIdRoutine =
            new SysIdRoutine(
                new SysIdRoutine.Config(
                    null,
                    null,
                    null,
                    (state) -> Logger.recordOutput("SysId/State", state.toString())
                ),
                new SysIdRoutine.Mechanism(
                    (voltage) -> {
                        DriveConstants.m_leftLeader.setVoltage(voltage);
                        DriveConstants.m_rightLeader.setVoltage(voltage);
                        m_drive.doFeed();
                    },

                     (SysIdRoutineLog log) -> {
                        log.motor("Drivetrain")
                            .voltage(
                                edu.wpi.first.units.Units.Volts.of(
                                    DriveConstants.m_leftLeader.getAppliedOutput()
                                        * DriveConstants.m_leftLeader.getBusVoltage()
                                )
                            )
                            .angularPosition(
                                edu.wpi.first.units.Units.Rotations.of(DriveConstants.m_leftEncoder.getPosition())
                            )
                            .angularVelocity(
                                edu.wpi.first.units.Units.RotationsPerSecond.of(DriveConstants.m_leftEncoder.getVelocity() / 60.0)
                            );
                    },

                 m_drive
            )
        );
    }

    /**
     * The command to run a SysID Quasistatic
     * @param direction robot direction
     * @return the command to run a quasistatic SysID
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
      return m_sysIdRoutine.quasistatic(direction);
    }

    /**
     * The command to run a SysID Dynamic
     * @param direction robot direction
     * @return the command to run a dynamic SysID
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
      return m_sysIdRoutine.dynamic(direction);
    }
}
