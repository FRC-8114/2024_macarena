package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import static frc.robot.Constants.ShooterFlywheelConstants.shooterRPM;

import java.util.List;
import java.util.Map;
import java.util.NoSuchElementException;
import java.util.Optional;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.ControlSys.launchpad;
import frc.robot.ControlSys.launchtrigger;
import frc.robot.generated.TunerConstants;

// import all subsystems
import frc.robot.subsystems.*;

public class RobotContainer {
  private double MaxSpeed = 4.572; // meters per second
  private double MaxAngularRate = 1.5 * Math.PI; // radians per sec

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0);
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;

  // Initialize Subsystems
  public final IntakePivot intakePivot = new IntakePivot();
  public final IntakeRollers intakeRollers = new IntakeRollers();
  public final ShooterFlywheel shooterFlywheel = new ShooterFlywheel();
  public final ShooterPivot shooterPivot = new ShooterPivot();
  public final Telescope telescope = new Telescope();
  public final Trap trap = new Trap();
  public final Winch winch = new Winch();

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  public launchpad launch = new launchpad();

  private launchtrigger[][] button = new launchtrigger[8][8];

  public void configureBindings() {
    // TODO: Fix positives / negatives
    // Drive Controls (Flip depending on alliance)
      var ally = DriverStation.getAlliance();
      if (ally.isPresent()) {
        if (ally.get() == Alliance.Blue) {
          drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
              drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive with negative Y (forward)
                  .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                  .withRotationalRate(joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
              ));
        }
        if (ally.get() == Alliance.Red) {
          drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
              drivetrain.applyRequest(() -> drive.withVelocityX(joystick.getLeftY() * MaxSpeed) // Drive with negative Y (forward)
                  .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with positive X (left)
                  .withRotationalRate(joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with positive X (left)
              ));
        }
      }
      else {
        drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
              drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive with negative Y (forward)
                  .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                  .withRotationalRate(joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
              ));
      }

    // == Xbox Controller ==
    // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    // joystick.b().whileTrue(drivetrain
    //     .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // joystick.povDown().onTrue(moveToPose(new Pose2d(15.201, 5.574, new Rotation2d(0))));

    // reset the field-centric heading on left bumper press
    button[1][0].onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
    // joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    drivetrain.registerTelemetry(logger::telemeterize);

    // Telescope Controls
    joystick.rightBumper().onTrue(telescope.setSpeedCommand(7)).onFalse(telescope.setSpeedCommand(0));
    joystick.leftBumper().onTrue(telescope.setSpeedCommand(-5)).onFalse(telescope.setSpeedCommand(-1));

    // == Launchpad ==
    button[0][0].whileTrue(drivetrain.applyRequest(() -> brake)); // Motor Brake
    // button[0][1].onTrue(winch.setSpeedCommand(11)).onFalse(winch.setSpeedCommand(0)); // Pull Winch
    // button[0][2]
    // button[0][3]
    button[1][1].whileTrue(shooterFlywheel.setSpeedFromShuffle()).onFalse(shooterFlywheel.stopFlywheels());
    button[0][1].whileTrue(shooterFlywheel.startFlywheels()).onFalse(shooterFlywheel.stopFlywheels()); // Start Flywheels // Stop Flywheels
    button[0][2].onTrue(Commands.parallel(intakePivot.intakeDown(), intakeRollers.intakeNote()));   // Set Intake Angle to Position to intake note and intake note
    button[1][2].onTrue(intakePivot.intakeWeUp());   // Set Intake Angle to position to feed note to shooter
    button[2][2].onTrue(intakeRollers.outtakeNote()); // Outake
    button[3][2].onTrue(intakePivot.intakeAmp());
    button[4][2].onTrue(intakeRollers.slowOuttakeNote());
    button[0][3].onTrue(shooterPivot.setAngleFromShuffle()); // Shuffle Angler

    button[3][0].onTrue(intakePivot.resetAngle());

    button[0][6].onTrue(winch.setSpeedCommand(3)).onFalse(winch.setSpeedCommand(0));
    button[1][6].onTrue(winch.setSpeedCommand(-3)).onFalse(winch.setSpeedCommand(0));

    // AutoAim
    final AprilTagFieldLayout field = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    try {
      switch (DriverStation.getAlliance().get()) {
        case Blue:
          button[0][4].onTrue(
              shooterPivot.autoAngleFromPose(curPose(), field.getTagPose(7).get()));

        case Red:
          button[0][4].onTrue(
              shooterPivot.autoAngleFromPose(curPose(), field.getTagPose(4).get()));
      }
    } catch (NoSuchElementException e) {
      System.err.println("Invalid alliance or tag pose idk!!!!");
      System.err.println(e.getCause());
    }

    //SysID
    // joystick.povUp().and(joystick.a()).onTrue(shooterFlywheel.sysIdQuasistatic(Direction.kForward));
    // joystick.povUp().and(joystick.b()).onTrue(shooterFlywheel.sysIdQuasistatic(Direction.kReverse));
    joystick.povUp().and(joystick.x()).whileTrue(trap.setVoltageCommand(-6)).onFalse(trap.setVoltageCommand(0));
    joystick.povUp().and(joystick.y()).whileTrue(trap.setVoltageCommand(6)).onFalse(trap.setVoltageCommand(0));

    // shooterPivot.setDefaultCommand(
    //   shooterPivot.setAngleFromShuffle()
    // );

    // joystick.leftTrigger().whileTrue(shooterFlywheel.startFlywheels()).onFalse(shooterFlywheel.stopFlywheels());
    // joystick.povRight().onTrue(Commands.parallel(intakePivot.intakeDown(), intakeRollers.intakeNote()));
    // joystick.povUp().onTrue(intakePivot.intakeWeUp());
    // joystick.povDown().onTrue(intakeRollers.outtakeNote());
    joystick.povLeft().onTrue(shooterPivot.setAngleFromShuffle());

    joystick.rightTrigger().whileTrue(shooterFlywheel.setSpeedCommand(1)).onFalse(shooterFlywheel.stopFlywheels());
  }

  public RobotContainer() {
    for (int row = 0; row < button.length; row++) {
      for (int col = 0; col < button[row].length; col++) {
        System.out.print(row + " " + col);
        button[col][row] = new launchtrigger(launch, row, col);
      }
    }

    NamedCommands.registerCommand("intake", this.getIntakeNote());
    NamedCommands.registerCommand("intakeBack", intakePivot.intakeWeUp().withTimeout(3));

  }

  public Pose2d curPose() {
    return drivetrain.getState().Pose;
  }

  public Command getIntakeNote() {
    return Commands.parallel(intakePivot.intakeDown(), intakeRollers.intakeNote());
  }

  public Command moveToPose(Pose2d targetPose) {

    // List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(targetPose);

    // Create the constraints to use while pathfinding
    PathConstraints constraints = new PathConstraints(
        2.0, 2.0,
        Units.degreesToRadians(180), Units.degreesToRadians(180));

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    Command pathfindingCommand = AutoBuilder.pathfindToPose(
    targetPose,
    constraints,
    0.0,
    0.0
    // Goal end velocity in meters/sec
     // Rotation delay distance in meters. This is how far the robot should travel
    // before attempting to rotate.
    );

    // PathPlannerPath path = new PathPlannerPath(bezierPoints, constraints,
    //     new GoalEndState(0.0, targetPose.getRotation()));

    return pathfindingCommand;
  }

  public Command getAutonomousCommand() {
    // Load the path we want to pathfind to and follow
    // Load a Choreo trajectory as a PathPlannerPath
    // PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory("Example Choreo Traj");

    // // Create the constraints to use while pathfinding. The constraints defined in
    // // the path will only be used for the path.
    // PathConstraints constraints = new PathConstraints(
    //     3.5,2.0,
    //     Units.degreesToRadians(540), Units.degreesToRadians(720));

    // // Since AutoBuilder is configured, we can use it to build pathfinding commands
    // Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
    //     path,
    //     constraints,
    //     1.0 // Rotation delay distance in meters. This is how far the robot should travel
    //         // before attempting to rotate.
    // );

    Command pathTest = AutoBuilder.buildAuto("New Auto");
    return pathTest;
  }
}
