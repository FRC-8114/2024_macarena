package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.TimedRobot;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
// import java.util.List;

public class PhotonTag implements Subsystem{
    
    PhotonCamera camera;
    AprilTagFieldLayout aprilTagFieldLayout;
    Transform3d camToRobot = new Transform3d();

    public PhotonTag(String CamName, AprilTagFieldLayout field, Transform3d xyCamRelationToRobot) {
        camera = new PhotonCamera(CamName);
        aprilTagFieldLayout = field;
        camToRobot = xyCamRelationToRobot;
    }

    public PhotonPipelineResult tagCheck() {
    
        var result = camera.getLatestResult();

        return result;
    //     if (result.hasTargets()) {
    //     // lis😊😊😊😊😊😊😊😊😊😊😊😊😊😊😊😊😊😊😊😊😊😊😊😊😊😊😊😊😊😊😊t of all AprilTags in view
    //     // List<PhotonTrackedTarget> list_targets = result.getTargets();

    //       PhotonTrackedTarget target = result.getBestTarget(); // Select the best AprilTag in view
      
    //       Pose3d goalAprilTag = aprilTagFieldLayout.getTagPose(
    //         target.getFiducialId() // The id of the AprilTag in view
    //     ).get(); // unwrap Option<>
      
    //     Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
    //         target.getBestCameraToTarget(),
    //         goalAprilTag,
    //         camToRobot
    //     );

    //     // double distanceToTarget = PhotonUtils.getDistanceToPose( // Takes pose2d, not pose3d
    //     //     robotPose.toPose2d(),
    //     //     goalAprilTag.toPose2d()
    //     // );

    //     // distanceToTarget


    //  }

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

    public double roboDistance(Pose2d roboPose, PhotonTrackedTarget target) {
        double distanceToTarget = PhotonUtils.getDistanceToPose( // Takes pose2d, not pose3d
            roboPose,
            aprilTagFieldLayout.getTagPose(target.getFiducialId()).get().toPose2d()
        );

        return distanceToTarget;
    }

}
