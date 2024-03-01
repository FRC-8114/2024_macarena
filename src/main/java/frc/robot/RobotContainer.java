package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.ControlSys.launchpad;
import frc.robot.ControlSys.launchtrigger;
import frc.robot.generated.TunerConstants;

// import all subsystems
import frc.robot.subsystems.*;

public class RobotContainer {
  private double MaxSpeed = 3.5; // meters per second
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

  public static final launchpad launch = new launchpad();

  private final launchtrigger[][] button = new launchtrigger[8][8];

  Optional<Alliance> ally = DriverStation.getAlliance();
  private GenericEntry shootAngle = Shuffleboard.getTab("FieldInfo").add("Shooter Angle", 0.0).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 65)).getEntry();

  private void configureBindings() {
    // TODO: Fix positives / negatives
    // Drive Controls (Flip depending on alliance)
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

    joystick.povDown().onTrue(moveToPose(new Pose2d(15.201, 5.574, new Rotation2d(0))));

    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    drivetrain.registerTelemetry(logger::telemeterize);

    // Telescope Controls
    joystick.rightBumper().onTrue(telescope.setSpeedCommand(11)).onFalse(telescope.setSpeedCommand(0));
    joystick.leftBumper().onTrue(telescope.setSpeedCommand(-11)).onFalse(telescope.setSpeedCommand(0));

    // == Launchpad ==
    button[0][0].onTrue(drivetrain.applyRequest(() -> brake)); // Motor Brake
    button[0][1].onTrue(winch.setSpeedCommand(11)).onFalse(winch.setSpeedCommand(0)); // Pull Winch
    // button[0][2]
    // button[0][3]
    button[0][5].onTrue(shooterFlywheel.startFlywheels()); // Start Flywheels
    button[1][5].onTrue(shooterFlywheel.stopFlywheels());  // Stop Flywheels
    button[0][6].onTrue(intakePivot.setAngleCommand(0));   // Set Intake Angle to Position to intake note
    button[1][6].onTrue(intakePivot.setAngleCommand(0));   // Set Intake Angle to position to feed note to shooter
    button[0][7].onTrue(intakeRollers.intakeNote());       // Intake Note (stops on limSwitch)
    button[1][7].onTrue(intakeRollers.outtakeNote());      // Outtake note (times out)

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
    // joystick.povUp().and(joystick.a()).onTrue(shooterPivot.sysIdQuasistaticForward());
    // joystick.povUp().and(joystick.b()).onTrue(shooterPivot.sysIdQuasistaticReverse());
    // joystick.povUp().and(joystick.x()).onTrue(shooterPivot.sysIdDynamicForward());
    // joystick.povUp().and(joystick.y()).onTrue(shooterPivot.sysIdDynamicReverse());

    shooterPivot.setDefaultCommand(
      shooterPivot.setAngleCommand(Units.degreesToRotations(shootAngle.getDouble(0.0)))
    );

    joystick.povRight().whileTrue(shooterPivot.setAngleCommand(Units.degreesToRotations(40)));

    // joystick.povRight().onTrue(shooterPivot.printEncoder());


  }

  public RobotContainer() {
    for (int row = 0; row < button.length; row++) {
      for (int col = 0; col < button[row].length; col++) {
        button[col][row] = new launchtrigger(launch, row, col);
      }
    }
    configureBindings();
  }

  public Pose2d curPose() {
    return drivetrain.getState().Pose;
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
    PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory("Example Choreo Traj");

    // Create the constraints to use while pathfinding. The constraints defined in
    // the path will only be used for the path.
    PathConstraints constraints = new PathConstraints(
        3.5,2.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720));

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
        path,
        constraints,
        3.0 // Rotation delay distance in meters. This is how far the robot should travel
            // before attempting to rotate.
    );
    return pathfindingCommand;
  }
}
