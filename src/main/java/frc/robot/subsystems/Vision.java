package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    private final PhotonCamera m_camera = new PhotonCamera("JerrysEye");
    private final PhotonPoseEstimator m_poseEstimator;
    private PhotonPipelineResult latestResult;

    // ! Change this!
    private static final Transform3d robotToCam = new Transform3d(
        new Translation3d(0.1, 0.0, 0.2), 
        new Rotation3d(0, 0, 0)
    );

    public Vision() {
        AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
        m_poseEstimator = new PhotonPoseEstimator(layout, robotToCam);
    }

    @Override
    public void periodic() {
        List<PhotonPipelineResult> results = m_camera.getAllUnreadResults();

        for (var result : results) {
            latestResult = result;

            Optional<EstimatedRobotPose> visionEstimation = m_poseEstimator.estimateCoprocMultiTagPose(result);

            if (visionEstimation.isEmpty()) {
                visionEstimation = m_poseEstimator.estimateLowestAmbiguityPose(result);
            }

            visionEstimation.ifPresent(estimation -> {
                Logger.recordOutput("Vision/EstimatedPose", estimation.estimatedPose.toPose2d());
                Logger.recordOutput("Vision/Timestamp", estimation.timestampSeconds);
            });
        }

        if (latestResult != null) {
            Logger.recordOutput("Vision/HasTarget", latestResult.hasTargets());
        }
    }

    public boolean hasTargets() {
        return latestResult != null && latestResult.hasTargets();
    }

    public PhotonTrackedTarget getBestTarget() {
        return hasTargets() ? latestResult.getBestTarget() : null;
    }
}