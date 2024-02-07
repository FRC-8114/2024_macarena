// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.PhotonTag;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final Field2d field = new Field2d();

  private RobotContainer m_robotContainer;

  PhotonTag april = new PhotonTag("arducam1", AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
      new Transform3d(new Translation3d(-0.3429, 0, 0.216), new Rotation3d(0, -0.436, 3.1415)));
  boolean usingTags = true;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    Shuffleboard.update();
    Shuffleboard.getTab("FieldInfo").add("Field2d", field).withWidget("Field");
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    if (usingTags) {
      var pipeLine = april.getEstimatedGlobalPose();
      if (pipeLine.isPresent()) {
        if ((pipeLine.get().strategy == PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR)
            || (pipeLine.get().strategy != PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
                && pipeLine.get().targetsUsed.get(0).getPoseAmbiguity() > 0.2)) {
          Pose2d curPos = pipeLine.get().estimatedPose.toPose2d();
          m_robotContainer.drivetrain.addVisionMeasurement(curPos, pipeLine.get().timestampSeconds);
        }
      }
    }
    field.setRobotPose(m_robotContainer.drivetrain.getState().Pose);
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    //m_robotContainer.drivetrain.seedFieldRelative(new Pose2d(14.72234058380127, 7.769551753997803, new Rotation2d(1.5707963267948966)));
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
