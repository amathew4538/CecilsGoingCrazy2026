package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Gyroscope;

public class Turn180 extends Command {
    private final DriveSubsystem m_drive;
    private final Gyroscope m_gyro;

    /**
     * The Turn180 Command in a seperate file
     * @param drive the drive subsytem
     * @param gyro the gyroscope
     */
    public Turn180(DriveSubsystem drive, Gyroscope gyro) {
        m_drive = drive;
        m_gyro = gyro;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        double target = MathUtil.inputModulus(m_gyro.getHeading() + 180, -180, 180);
        DriveConstants.m_pid.setSetpoint(target);
    }

    @Override
    public void execute() {
        double rotationSpeed = DriveConstants.m_pid.calculate(m_gyro.getHeading());
        rotationSpeed = MathUtil.clamp(rotationSpeed, -0.75, 0.75);
        m_drive.arcadeDrive(0, rotationSpeed);
    }

    @Override
    public boolean isFinished() {
        return DriveConstants.m_pid.atSetpoint();
    }
}
