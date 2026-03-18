package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class OdometryManager extends SubsystemBase{
    private final Gyroscope m_gyro;

    private final DifferentialDriveOdometry m_odometry;
    private final DifferentialDrivePoseEstimator m_poseEstimator;

    public OdometryManager(Gyroscope gyro) {
        this.m_gyro = gyro;

        this.m_odometry = new DifferentialDriveOdometry(
          Rotation2d.fromDegrees(m_gyro.getHeading()),
          0.0,
          0.0
        );

        this.m_poseEstimator = new DifferentialDrivePoseEstimator(
            DriveConstants.m_kinematics,
            Rotation2d.fromDegrees(m_gyro.getHeading()),
            0.0,
            0.0,
            new Pose2d()
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
        m_gyro.resetHeading();

        DriveConstants.m_leftEncoder.setPosition(0);
        DriveConstants.m_rightEncoder.setPosition(0);

        DriveConstants.m_prevLeftDist = 0;
        DriveConstants.m_prevRightDist = 0;
        DriveConstants.m_totalLeftDist = 0;
        DriveConstants.m_totalRightDist = 0;

        m_odometry.resetPosition(Rotation2d.fromDegrees(m_gyro.getHeading()), 0, 0, pose);
        m_poseEstimator.resetPosition(Rotation2d.fromDegrees(m_gyro.getHeading()), 0, 0, pose);
    }

    /**
     * Adds vision to odometry
     * @param visionRobotPose the pose according to vision
     * @param timestamp the timestamp from vision
     */
    public void addVisionMeasurement(Pose2d visionRobotPose, double timestamp) {
        m_poseEstimator.addVisionMeasurement(visionRobotPose, timestamp);
    }

    /**
     * Gets the combined pose of the robot
     * @return the pose
     */
    public Pose2d getFinalPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    @Override
    public void periodic() {
        m_odometry.update(
            Rotation2d.fromDegrees(m_gyro.getHeading()),
            DriveConstants.m_totalLeftDist,
            DriveConstants.m_totalRightDist
        );
        m_poseEstimator.update(
            Rotation2d.fromDegrees(m_gyro.getHeading()),
            DriveConstants.m_totalLeftDist,
            DriveConstants.m_totalRightDist
        );
        DriveConstants.m_Field2d.setRobotPose(getFinalPose());

        Logger.recordOutput("Odometry/FinalPose", getFinalPose());
        Logger.recordOutput("Odometry/FinalPose3d", new Pose3d(getFinalPose()));
    }
}
