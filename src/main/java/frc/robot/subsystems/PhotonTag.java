package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.*;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import static frc.robot.Constants.photonConstants.*;

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

    public Matrix<N3, N1> getSTDDEVS(Optional<EstimatedRobotPose> pipe) {
        var estStdDevs = singleTagSTDDEVS;
        var estPose = pipe.get().estimatedPose.toPose2d();
        var targets = pipe.get().targetsUsed;
        int numTags = 0;
        double avgDist = 0;

        for (var targ : targets) {
            var tagPose = poseEstimator.getFieldTags().getTagPose(targ.getFiducialId());
            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estPose.getTranslation());
        }

        if (numTags == 0) return estStdDevs;
        avgDist /= numTags;
        if (numTags > 1) estStdDevs = multiTagSTDDEVS;

        estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        return estStdDevs;
    }

    public void update() {

    }

}
