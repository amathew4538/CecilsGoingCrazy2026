package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class OdometryManager extends SubsystemBase{
    private final Gyroscope m_gyro;

    private final DifferentialDriveOdometry m_odometry;

    public OdometryManager(Gyroscope gyro) {
        this.m_gyro = gyro;

        this.m_odometry = new DifferentialDriveOdometry(
          Rotation2d.fromDegrees(m_gyro.getHeading()),
          0.0,
          0.0
        );
    }

    /**
     * Gets the pose of the robot from the odometry
     * @return The pose of the robot as a Pose2D
    */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Resets the odometry of the robot
     * @param pose Pose of the robot
    */
    public void resetOdometry(Pose2d pose) {
        DriveConstants.m_leftEncoder.setPosition(0);
        DriveConstants.m_rightEncoder.setPosition(0);

        DriveConstants.m_prevLeftDist = 0;
        DriveConstants.m_prevRightDist = 0;
        DriveConstants.m_totalLeftDist = 0;
        DriveConstants.m_totalRightDist = 0;

        m_odometry.resetPosition(Rotation2d.fromDegrees(m_gyro.getHeading()), 0, 0, pose);
    }

    @Override
    public void periodic() {
        m_odometry.update(
            Rotation2d.fromDegrees(m_gyro.getHeading()),
            DriveConstants.m_totalLeftDist,
            DriveConstants.m_totalRightDist
        );

        var currentPose = getPose();
        DriveConstants.m_Field2d.setRobotPose(currentPose);
    }
}
