// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.TimedRobot;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Robot extends TimedRobot {
  // TODO: Get Camera's name
  PhotonCamera camera = new PhotonCamera("Camera Name");
  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  Transform3d camToRobot = new Transform3d(); // Should specify position of camera in relation to the robot

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    var result = camera.getLatestResult();

    if (result.hasTargets()) {
      // list of all AprilTags in view
      // List<PhotonTrackedTarget> list_targets = result.getTargets();

      PhotonTrackedTarget target = result.getBestTarget(); // Select the best AprilTag in view
      
      Pose3d goalAprilTag = aprilTagFieldLayout.getTagPose(
        target.getFiducialId() // The id of the AprilTag in view
      ).get(); // unwrap Option<>
      
      Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
        target.getBestCameraToTarget(),
        goalAprilTag,
        camToRobot
      );

      double distanceToTarget = PhotonUtils.getDistanceToPose( // Takes pose2d, not pose3d
        robotPose.toPose2d(),
        goalAprilTag.toPose2d()
      );

      System.out.println(distanceToTarget);
    }
  }
}
