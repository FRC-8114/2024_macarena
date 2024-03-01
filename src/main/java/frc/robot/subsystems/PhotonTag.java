package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.*;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonTag implements Subsystem {
    
    PhotonCamera camera;
    AprilTagFieldLayout aprilTagFieldLayout;
    Transform3d camToRobot = new Transform3d();
    PhotonPoseEstimator poseEstimator;

    public PhotonTag(String CamName, AprilTagFieldLayout field, Transform3d xyCamRelationToRobot) {
        camera = new PhotonCamera(CamName);
        aprilTagFieldLayout = field;
        camToRobot = xyCamRelationToRobot;
        poseEstimator = new PhotonPoseEstimator(field, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, xyCamRelationToRobot);
        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    public PhotonPipelineResult tagCheck() {
    
        var result = camera.getLatestResult();

        return result;
    }

    public Pose2d getRoboPose(PhotonTrackedTarget target) {
        
        Pose3d goalAprilTag = aprilTagFieldLayout.getTagPose(
            target.getFiducialId() // The id of the AprilTag in view
        ).get(); // unwrap Option<>
      
        Pose3d roboPose = PhotonUtils.estimateFieldToRobotAprilTag(
            target.getBestCameraToTarget(),
            goalAprilTag,
            camToRobot
        );

        return roboPose.toPose2d();
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        return poseEstimator.update();
    }

    public double roboDistance(Pose2d roboPose, PhotonTrackedTarget target) {
        double distanceToTarget = PhotonUtils.getDistanceToPose( // Takes pose2d, not pose3d
            roboPose,
            aprilTagFieldLayout.getTagPose(target.getFiducialId()).get().toPose2d()
        );

        return distanceToTarget;
    }

}
