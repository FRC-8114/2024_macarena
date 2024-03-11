package frc.robot;

import java.util.Optional;

import org.photonvision.PhotonUtils;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.ControlSys.launchpad;
import frc.robot.ControlSys.launchtrigger;
import frc.robot.generated.TunerConstants;
// import all subsystems
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.ShooterFlywheel;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.Telescope;
import frc.robot.subsystems.Winch;

public class RobotContainer {
  private double MaxSpeed = 4.572; // meters per second
  private double MaxAngularRate = 1.5 * Math.PI; // radians per sec

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0);
  public static final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;

  // Initialize Subsystems
  public final IntakePivot intakePivot = new IntakePivot();
  public final IntakeRollers intakeRollers = new IntakeRollers();
  public final ShooterFlywheel shooterFlywheel = new ShooterFlywheel();
  public final ShooterPivot shooterPivot = new ShooterPivot();
  public final Telescope telescope = new Telescope();
  // public final Trap trap = new Trap();
  public final Winch winch = new Winch();

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // field-centric driving in open loop

  private final SwerveRequest.FieldCentricFacingAngle angleDrive = new SwerveRequest.FieldCentricFacingAngle()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  public launchpad launchpad = new launchpad();

  private launchtrigger[][] button = new launchtrigger[8][8];
  private double turtleMode = 1.0;

  DriverStation.Alliance ally = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : DriverStation.Alliance.Blue;

  public void configureBindings() {
    // Drive Controls (Flip depending on alliance)
    switch (ally) {
      case Blue:
        drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * turtleMode) // Drive with negative Y (forward)
          .withVelocityY(-joystick.getLeftX() * MaxSpeed * turtleMode) // Drive left with negative X (left)
          .withRotationalRate(-joystick.getRightX() * MaxAngularRate * turtleMode) // Drive counterclockwise with negative X (left)
        ));
      case Red:
        drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(joystick.getLeftY() * MaxSpeed * turtleMode) // Drive with negative Y (forward)
          .withVelocityY(joystick.getLeftX() * MaxSpeed * turtleMode) // Drive left with positive X (left)
          .withRotationalRate(-joystick.getRightX() * MaxAngularRate * turtleMode) // Drive counterclockwise with positive X (left)
        ));
    }

    // == Xbox Controller ==
    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // joystick.povDown().onTrue(moveToPose(new Pose2d(15.201, 5.574, new Rotation2d(0))));

    // reset the field-centric heading on left bumper press
    button[1][0].onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
    // joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    drivetrain.registerTelemetry(logger::telemeterize);


    shooterPivot.setDefaultCommand(shooterPivot.setAngle(60));

    // Telescope Controls
    joystick.rightBumper().onTrue(telescope.setSpeedCommand(7)).onFalse(telescope.setSpeedCommand(0));
    joystick.leftBumper().onTrue(telescope.setSpeedCommand(-5)).onFalse(telescope.setSpeedCommand(-1));

    joystick.leftTrigger().onTrue(intakePivot.intakeWeUp()).whileTrue(shooterPivot.setAngle(37));
    joystick.povDown().onTrue(Commands.parallel(intakePivot.intakeDown(), intakeRollers.intakeNote())).whileTrue(shooterPivot.setAngle(37));
    joystick.povRight().whileTrue(shooterAngle());
    joystick.rightTrigger().onTrue(this.shotSequence());
    joystick.povUp().onTrue(Commands.runOnce(() -> {
      if (turtleMode == 1) {
        turtleMode = 0.25;
      }
      else 
        turtleMode = 1.0;
    }));

    // == Launchpad ==
    button[0][0].whileTrue(drivetrain.applyRequest(() -> brake)); // Motor Brake
    button[3][1].onTrue(shooterFlywheel.stopFlywheels());
    button[2][1].onTrue(shooterFlywheel.startFlywheels()); // Start Flywheels // Stop Flywheels
    button[0][2].onTrue(Commands.parallel(intakePivot.intakeDown(), intakeRollers.intakeNote())).whileTrue(shooterPivot.setAngle(37));   // Set Intake Angle to Position to intake note and intake note
    button[1][2].onTrue(Commands.parallel(intakeRollers.intakeStop(), intakePivot.intakeWeUp())).whileTrue(shooterPivot.setAngle(37));   // Set Intake Angle to position to feed note to shooter
    button[3][2].onTrue(intakeRollers.outtakeNote()); // Outake
    button[2][2].onTrue(this.shotSequence());
    button[4][2].onTrue(intakeRollers.slowOuttakeNote());
    button[5][2].onTrue(intakePivot.intakeSetVoltage(-4)).onFalse(intakePivot.stopMotor());

    button[0][5].onTrue(shooterPivot.setAngleFromShuffle()); // Shuffle Angler

    button[3][0].onTrue(intakePivot.resetAngle());
    button[3][3].whileTrue(shooterPivot.setAngle(37)).onTrue(Commands.sequence(intakePivot.intakeAmp(), intakeRollers.slowOuttakeNote()));

    button[0][6].onTrue(winch.setSpeedCommand(3)).onFalse(winch.setSpeedCommand(0));
    button[1][6].onTrue(winch.setSpeedCommand(-3)).onFalse(winch.setSpeedCommand(0));

    // AutoAim
    button[4][1].onTrue(shooterFlywheel.slowFlywheels(925)).onFalse(shooterFlywheel.stopFlywheels());
    button[2][3].whileTrue(shooterAngle());
    button[0][3].whileTrue(shooterPivot.setAngle(37));
    button[0][4].whileTrue(shooterPivot.setAngleFromShuffle());

    // SysID
    // joystick.povUp().and(joystick.a()).onTrue(shooterFlywheel.sysIdQuasistatic(Direction.kForward));
    // joystick.povUp().and(joystick.b()).onTrue(shooterFlywheel.sysIdQuasistatic(Direction.kReverse));
    // joystick.x().whileTrue(trap.setVoltageCommand(-6)).onFalse(trap.setVoltageCommand(0));
    // joystick.y().whileTrue(trap.setVoltageCommand(6)).onFalse(trap.setVoltageCommand(0));

    // shooterPivot.setDefaultCommand(
    //   shooterPivot.setAngleFromShuffle()
    // );

    // joystick.leftTrigger().whileTrue(shooterFlywheel.startFlywheels()).onFalse(shooterFlywheel.stopFlywheels());
    // joystick.povRight().onTrue(Commands.parallel(intakePivot.intakeDown(), intakeRollers.intakeNote()));
    // joystick.povUp().onTrue(intakePivot.intakeWeUp());
    // joystick.povDown().onTrue(intakeRollers.outtakeNote());
    // joystick.povLeft().onTrue(shooterPivot.setAngleFromShuffle());

    // joystick.rightTrigger().whileTrue(shooterFlywheel.setSpeedCommand(1)).onFalse(shooterFlywheel.stopFlywheels());
  }

  public RobotContainer() {
    for (int row = 0; row < button.length; row++) {
      for (int col = 0; col < button[row].length; col++) {
        System.out.print(row + " " + col);
        button[col][row] = new launchtrigger(launchpad, row, col);
      }
    }

    NamedCommands.registerCommand("intake", intakePivot.intakeDown());
    NamedCommands.registerCommand("intakeRollIn", intakeRollers.intakeNote().withTimeout(2));
    NamedCommands.registerCommand("intakeBack", intakePivot.intakeWeUp());
    NamedCommands.registerCommand("outtake", this.shotSequence());
    NamedCommands.registerCommand("quickShot", Commands.waitUntil(shooterPivot::atSetpoint).andThen(intakeRollers.outtakeNote().andThen(shooterFlywheel.stopFlywheels())));
    NamedCommands.registerCommand("shooterStart", this.shooterStart());
    NamedCommands.registerCommand("shooterStop", this.shooterStop());
    NamedCommands.registerCommand("flywheelStart", shooterFlywheel.setSpeedNoStop(6300));
    NamedCommands.registerCommand("shooterLoadPosition", shooterPivot.setAngle(37));
    NamedCommands.registerCommand("feedNote", Commands.waitUntil(shooterPivot::atSetpoint).andThen(intakePivot.intakeWeUp()));
    NamedCommands.registerCommand("shooterAim", shooterAngle());
    NamedCommands.registerCommand("shootNote", Commands.waitUntil(shooterPivot::atSetpoint).andThen(this.shotSequence()));
    NamedCommands.registerCommand("startAim", shooterPivot.setAngle(57));
    NamedCommands.registerCommand("ampAim", shooterPivot.setAngle(59));
    NamedCommands.registerCommand("ampShot", shooterFlywheel.slowFlywheels(925).withTimeout(3));
    NamedCommands.registerCommand("waitedOuttake", Commands.waitUntil(shooterPivot::atSetpoint).andThen(Commands.waitSeconds(.5).andThen(intakeRollers.outtakeNote())));

    auton.setDefaultOption("4 Note", AutoBuilder.buildAuto("4 note auto"));
    auton.addOption("4 Note", AutoBuilder.buildAuto("4 note auto"));
    auton.addOption("drive back 2", AutoBuilder.buildAuto("drive back"));
    auton.addOption("1 note bottom", AutoBuilder.buildAuto("1 note preload bottom"));
    auton.addOption("1 note top", AutoBuilder.buildAuto("1 note preload top"));
    auton.addOption("far top 2", AutoBuilder.buildAuto("far top 2"));
    auton.addOption("1 far note bottom", AutoBuilder.buildAuto("1 far bottom"));
    auton.addOption("amp 2", AutoBuilder.buildAuto("amp"));

    Shuffleboard.getTab("FieldInfo").add(auton);
  }

  public Pose2d curPose() {
    return drivetrain.getState().Pose;
  }

  public Command getIntakeNote() {
    return Commands.parallel(intakePivot.intakeDown(), intakeRollers.intakeNote().withTimeout(2));
  }

  public Command ampShot() {
    return intakePivot.intakeAmp().andThen(intakeRollers.slowOuttakeNote());
  }
  
  public Command shooterStart() {
    return Commands.sequence(shooterPivot.setAngle(36).withTimeout(1), this.shooterAngle().withTimeout(.5));
  }

  public Command lowBack() {
    return Commands.parallel(intakePivot.intakeWeUp(), shooterPivot.setAngle(36)).withTimeout(1);
  }

  public Command shotSequence() {
    return Commands.deadline(Commands.waitSeconds(1.1).andThen(intakeRollers.outtakeNote()), (shooterFlywheel.startFlywheels().withTimeout(3).andThen(shooterFlywheel.stopFlywheels())));
  }

  public Command shooterStop() {
    return Commands.parallel(shooterFlywheel.stopFlywheels(), shooterPivot.setAngle(59));
  }

  final AprilTagFieldLayout field = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  
  public Command shooterAngle() {
    switch (ally) {
      case Red:  return shooterPivot.autoAngle(field.getTagPose(7).get());
      case Blue: return shooterPivot.autoAngle(field.getTagPose(4).get());
      default:   return shooterPivot.autoAngle(field.getTagPose(4).get());
    }
  }

  public Rotation2d getShotAngle() {
    return PhotonUtils.getYawToPose(curPose(), field.getTagPose(4).get().toPose2d());
  }

  public Command driveAngle() {
    System.out.println("" + PhotonUtils.getYawToPose(curPose(), field.getTagPose(4).get().toPose2d()));
    return drivetrain.applyRequest(() -> angleDrive.withVelocityX(joystick.getLeftY() * MaxSpeed) // Drive with negative Y (forward)
                  .withVelocityY(joystick.getLeftX() * MaxSpeed) // Drive left with positive X (left)
                  .withTargetDirection(getShotAngle()) // Drive counterclockwise with positive X (left)
              );
  }

  public Command moveToPose(Pose2d targetPose) {
    // Create the constraints to use while pathfinding
    PathConstraints constraints = new PathConstraints(
        2.0, 2.0,
        Units.degreesToRadians(180), Units.degreesToRadians(180));

    // can use AutoBuilder to build pathfinding commands
    Command pathfindingCommand = AutoBuilder.pathfindToPose(
      targetPose,
      constraints,
      0.0, // meters/sec
      0.0 // meters. This is how far the robot should travel before attempting to rotate.
    );

    return pathfindingCommand;
  }

  SendableChooser<Command> auton = new SendableChooser<>();

  public Command getAutonomousCommand() {
    // Load the path we want to pathfind to and follow
    // Load a Choreo trajectory as a PathPlannerPath
    // PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory("Example Choreo Traj");

    // // Create the constraints to use while pathfinding. The constraints defined in
    // // the path will only be used for the path.
    // PathConstraints constraints = new PathConstraints(
    //     3.5,2.0,
    //     Units.degreesToRadians(540), Units.degreesToRadians(720));

    // // Sice AutoBuilder is configured, we can use it to build pathfinding commands
    // Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
    //     path,
    //     constraints,
    //     1.0 // Rotation delay distance in meters. This is how far the robot should travel
    //         // before attempting to rotate.
    // );
    intakePivot.resetAngl();

    //return Commands.parallel(pathTest, this.shooterAngle().asProxy());
    return auton.getSelected();
  }
}
