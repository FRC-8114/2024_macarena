package frc.robot.subsystems;

import static frc.robot.Constants.ShooterPivotConstants.*;

import java.util.Map;

import org.photonvision.PhotonUtils;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class ShooterPivot extends SubsystemBase {
    private InterpolatingDoubleTreeMap angleMap = new InterpolatingDoubleTreeMap();
    private final TalonFX shooterPivot = new TalonFX(shooterPivotID, "canivore");
    private final CANcoder shooterPivotEncoder = new CANcoder(4, "canivore");
    private final PositionVoltage pos = new PositionVoltage(0, 0, true, 0, 0, false, false, false);
    private final TrapezoidProfile profile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration)
    );
    
    private MotorOutputConfigs talonMfg() {
        MotorOutputConfigs mfg = new MotorOutputConfigs();
        mfg.NeutralMode = NeutralModeValue.Brake;
        mfg.Inverted = InvertedValue.CounterClockwise_Positive;
        return mfg;
    }

    private TalonFXConfiguration talonCfg(CANcoder cancoder) {
        TalonFXConfiguration talon_cfg = new TalonFXConfiguration();

        // FeedbackConfigs talon_cfg.Feedback = talon_cfg.Feedback;
        talon_cfg.Feedback.FeedbackRemoteSensorID = cancoder.getDeviceID();
        talon_cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        talon_cfg.Feedback.RotorToSensorRatio = gearRatio;
        talon_cfg.Feedback.SensorToMechanismRatio = 1.0;
        talon_cfg.Voltage.PeakForwardVoltage = 12;
        talon_cfg.Voltage.PeakReverseVoltage = -12;

        // talon_cfg.Slot0Configs talon_cfg.Slot0 = talon_cfg.talon_cfg.Slot0;
        talon_cfg.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        talon_cfg.Slot0.kP = kP; talon_cfg.Slot0.kI = kI; talon_cfg.Slot0.kD = kD;
        talon_cfg.Slot0.kV = kV; talon_cfg.Slot0.kS = kS; talon_cfg.Slot0.kG = kG;
        return talon_cfg;
    }

    private MagnetSensorConfigs encoderCfg() {
        MagnetSensorConfigs encoder_cfg = new MagnetSensorConfigs();
        encoder_cfg.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        encoder_cfg.MagnetOffset = encoderMagnentOffset;
        return encoder_cfg;
    }

    private void generateAngleMap() {
        angleMap.put(Units.inchesToMeters(16+38.5), 59.59);
        angleMap.put(Units.inchesToMeters(16+12+38.5), 54.97);
        angleMap.put(Units.inchesToMeters(16+24+38.5), 48.19);
        angleMap.put(Units.inchesToMeters(16+36+38.5), 47.97);
        angleMap.put(Units.inchesToMeters(16+48+38.5), 45.68);
        angleMap.put(Units.inchesToMeters(16+60+38.5), 44.20);
        angleMap.put(Units.inchesToMeters(16+72+38.5), 41.50);
        angleMap.put(Units.inchesToMeters(16+84+38.5), 41.275);
        angleMap.put(Units.inchesToMeters(16+96+38.5), 40.40);
        angleMap.put(Units.inchesToMeters(16+108+38.5), 38.89);
    }
    
    public ShooterPivot() {
        generateAngleMap();
        shooterPivotEncoder.getConfigurator().apply(encoderCfg());
        shooterPivot.getConfigurator().apply(talonCfg(shooterPivotEncoder));
        shooterPivot.getConfigurator().apply(talonMfg());
    }
    
    public double getAngle() {
        return shooterPivot.getPosition().getValueAsDouble();
    }

    public Command setAngle(double goalAngle) {
        // TrapezoidProfile.State m_goal = new TrapezoidProfile.State(Units.degreesToRotations(goalAngle), 0);
        // TrapezoidProfile.State curPoint = new TrapezoidProfile.State(getAngle(), shooterPivot.getVelocity().getValueAsDouble());
        // double start = Timer.getFPGATimestamp();
        // System.out.println("Shooter Rotation Start");

        return run(() -> {
            // TrapezoidProfile.State calc = profile.calculate(Timer.getFPGATimestamp()-start, curPoint, m_goal);
            // System.out.println("Pos: " + Units.degreesToRotations(goalAngle) + " | CurPose: " + this.getAngle());
            shooterPivot.setControl(
                pos.withPosition(Units.degreesToRotations(goalAngle))
            );
        });
    }

    // public Command setAngleFromShuffle() {
    //     Shuffleboard.update();
    //     TrapezoidProfile.State m_goal = new TrapezoidProfile.State(Units.degreesToRotations(shuffleAngle.getDouble(0)), 0);
    //     TrapezoidProfile.State curPoint = new TrapezoidProfile.State(getAngle(), shooterPivot.getVelocity().getValueAsDouble());
    //     double start = Timer.getFPGATimestamp();
    //     // System.out.println("Shooter Rotation Start");

    //     return run(() -> {
    //         TrapezoidProfile.State calc = profile.calculate(Timer.getFPGATimestamp()-start, curPoint, m_goal);
    //         // System.out.println("Pos: " + calc.position + " | " + calc.velocity);
    //         shooterPivot.setControl(
    //             pos.withPosition(calc.position).withVelocity(calc.velocity)
    //         );
    //     });
    // }
    final AprilTagFieldLayout field = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    Pose2d pose2d;
    public Command setAngleFromPose() {
        var dsAlliance = DriverStation.getAlliance();
        pose2d = field.getTagPose(7).get().toPose2d();
    // Drive Controls (Flip depending on alliance)
        if (dsAlliance.isPresent()) {
            if (dsAlliance.get() == DriverStation.Alliance.Blue) {
                pose2d = field.getTagPose(7).get().toPose2d();
            }
            else if (dsAlliance.get() == DriverStation.Alliance.Red) {
                pose2d = field.getTagPose(4).get().toPose2d();
            }
        }

        return run(() -> {
            // System.out.println("" + getShotAngle(goalPose) + " | " + PhotonUtils.getDistanceToPose(RobotContainer.drivetrain.getState().Pose, goalPose.toPose2d()));
            double position = Units.degreesToRotations(angleMap.get(Math.abs(PhotonUtils.getDistanceToPose(RobotContainer.drivetrain.getState().Pose, pose2d))));
            shooterPivot.setControl(pos.withPosition(position));
        }
        );
    }

    private GenericEntry shuffleAngle = Shuffleboard.getTab("FieldInfo")
        .add("Shooter Angle", 0.0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of(
            "min", kMinRotation,
            "max", kMaxRotation
        ))
        .getEntry();

    public Command setAngleFromShuffle() {
        Shuffleboard.update();
        return setAngle(shuffleAngle.getDouble(0));
    }

    public Command stopMotors() {
        return runOnce(() -> shooterPivot.set(0));
    }

    double tolerance = Units.degreesToRotations(3.0);
    public boolean atSetpoint() {
        return Math.abs(shooterPivot.getClosedLoopError().getValueAsDouble()) < tolerance;
    }

    // public Command printEncoder(Pose2d currentPose, Pose3d goalPose) {
    // }

    // // SysID
    // private final MutableMeasure<Voltage> appliedVoltage = mutable(Volts.of(0));
    // // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    // private final MutableMeasure<Angle> distance = mutable(Rotations.of(0));
    // // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    // private final MutableMeasure<Velocity<Angle>> velocity = mutable(RotationsPerSecond.of(0));

    // public void consumeSysIdVoltage(Measure<Voltage> voltage) {
    //     shooterPivot.setVoltage(voltage.in(Volts));
    // }

    // private void consumeSysIdLog(SysIdRoutineLog log) {
    //     log.motor("Shooter Pivot")
    //         .voltage(
    //             appliedVoltage.mut_replace(
    //                 shooterPivot.getAppliedOutput() * shooterPivot.getBusVoltage(), Volts))
    //         .angularPosition(distance.mut_replace(shooterPivotEncoder.getPosition().getValueAsDouble(), Rotations))
    //         .angularVelocity(
    //             velocity.mut_replace(shooterPivotEncoder.getVelocity().getValueAsDouble(), RotationsPerSecond));
    // }

    // SysIdRoutine routine = new SysIdRoutine(
    //     new SysIdRoutine.Config(Volts.of(0.5).per(Seconds), Volts.of(2), null),
    //     new SysIdRoutine.Mechanism(
    //         this::consumeSysIdVoltage, // Set voltage method
    //         this::consumeSysIdLog, // Log function
    //         this
    //     )
    // );

    // private double upper_bound = Units.degreesToRotations(60);
    // private double lower_bound = Units.degreesToRotations(0);

    // public Command sysIdQuasistaticForward() {
    //     return routine.quasistatic(Direction.kForward).until(() -> (shooterPivotEncoder.getPosition().getValueAsDouble() <= lower_bound)).andThen(stopMotors());
    // }

    // public Command sysIdQuasistaticReverse() {
    //     return routine.quasistatic(Direction.kReverse).until(() -> (shooterPivotEncoder.getPosition().getValueAsDouble() >= upper_bound)).andThen(stopMotors());
    // }

    // public Command sysIdDynamicForward() {
    //     return routine.dynamic(Direction.kForward).until(() -> (shooterPivotEncoder.getPosition().getValueAsDouble() <= lower_bound)).andThen(stopMotors());
    // }

    // public Command sysIdDynamicReverse() {
    //     return routine.dynamic(Direction.kReverse).until(() -> (shooterPivotEncoder.getPosition().getValueAsDouble() >= upper_bound)).andThen(stopMotors());
    // }
}