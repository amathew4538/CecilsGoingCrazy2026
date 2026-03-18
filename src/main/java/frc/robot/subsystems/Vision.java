package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    private final PhotonCamera m_camera = new PhotonCamera("JerrysEye");
    private final PhotonPoseEstimator m_poseEstimator;
    private PhotonPipelineResult latestResult;

    private VisionSystemSim m_visionSim;
    private PhotonCameraSim m_cameraSim;

    // ! Change this!
    private static final Transform3d robotToCam = new Transform3d(
        new Translation3d(0.1, 0.0, 0.2), 
        new Rotation3d(0, 0, 0)
    );

    public Vision() {
        AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
        m_poseEstimator = new PhotonPoseEstimator(layout, robotToCam);

        if (RobotBase.isSimulation()) {
            var cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(68.5));
            cameraProp.setCalibError(0.01, 0.08);
            cameraProp.setFPS(30);
            cameraProp.setAvgLatencyMs(35);       // USB cameras usually have ~35ms latency
            cameraProp.setLatencyStdDevMs(5);

            m_visionSim = new VisionSystemSim("main");
            m_visionSim.addAprilTags(layout);

            m_cameraSim = new PhotonCameraSim(m_camera, cameraProp);
            m_cameraSim.enableRawStream(true);
            m_cameraSim.enableProcessedStream(true);

            m_visionSim.addCamera(m_cameraSim, robotToCam);
        }

        if (RobotBase.isSimulation()) {
            var debugField = m_visionSim.getDebugField();
            Shuffleboard.getTab("Vision").add("SimField", debugField)
                .withWidget(BuiltInWidgets.kField);
        }
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
                if (estimation.targetsUsed.get(0).getPoseAmbiguity() < 0.2) {
                    Logger.recordOutput("Robot/Vision/EstimatedPose", estimation.estimatedPose.toPose2d());
                    Logger.recordOutput("Robot/Vision/EstimatedPose3d", estimation.estimatedPose);
                    Logger.recordOutput("Robot/Vision/Timestamp", estimation.timestampSeconds);
                }
            });
        }
    }

    public boolean hasTargets() {
        return latestResult != null && latestResult.hasTargets();
    }

    public PhotonTrackedTarget getBestTarget() {
        return hasTargets() ? latestResult.getBestTarget() : null;
    }

    /**
     * This method connects the physics sim pose to the vision sim
     */
    public void updateSimPose(Pose2d robotPose) {
        if (RobotBase.isSimulation()) {
            m_visionSim.update(robotPose);
        }
    }
}