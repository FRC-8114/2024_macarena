// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.PhotonTag;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final Field2d field = new Field2d();

  private RobotContainer m_robotContainer;

  PhotonTag april = new PhotonTag(
    "arducam1",
    AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
    new Transform3d(new Translation3d(-0.3429, 0, 0.216),
    new Rotation3d(0, -0.436, 3.1415)));
  public static boolean usingTags = true;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    Shuffleboard.update();
    Shuffleboard.getTab("FieldInfo").add("Field2d", field).withWidget("Field");

    SignalLogger.stop();
    SignalLogger.enableAutoLogging(false);
    // DataLogManager.start();
    // URCL.start();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    var pipeline = april.getEstimatedGlobalPose();
    if (usingTags && pipeline.isPresent()) {
      Pose2d curPos = pipeline.get().estimatedPose.toPose2d();
      if ((pipeline.get().strategy == PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR)
          || pipeline.get().targetsUsed.get(0).getPoseAmbiguity() > 0.3) {
        m_robotContainer.drivetrain.addVisionMeasurement(curPos, pipeline.get().timestampSeconds, april.getSTDDEVS(pipeline));
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
    usingTags = false;
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
    usingTags = true;
    m_robotContainer.resetIntake();
    m_robotContainer.configureBindings();
    SignalLogger.stop();
    // m_robotContainer.drivetrain.seedFieldRelative(new Pose2d(14.72234058380127, 7.769551753997803, new Rotation2d(1.5707963267948966)));
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
