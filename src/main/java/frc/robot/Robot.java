// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
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

  PhotonTag april = new PhotonTag("arducam1", AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(), new Transform3d(0.356, 0, 0.178, new Rotation3d(0,0,180)));
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
        if (april.getEstimatedGlobalPose(m_robotContainer.drivetrain.getState().Pose).isPresent()) {
          Pose2d curPos = april.getEstimatedGlobalPose(m_robotContainer.drivetrain.getState().Pose).get().estimatedPose.toPose2d();
          System.out.println("tag seen");
          m_robotContainer.drivetrain.addVisionMeasurement(curPos, Timer.getFPGATimestamp());
        }
    }
    field.setRobotPose(m_robotContainer.drivetrain.getState().Pose);
    Shuffleboard.update();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.drivetrain.seedFieldRelative(new Pose2d(14.72234058380127, 7.769551753997803, new Rotation2d(1.5707963267948966)));
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
