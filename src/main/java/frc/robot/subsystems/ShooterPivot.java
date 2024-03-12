package frc.robot.subsystems;

import static frc.robot.Constants.ShooterPivotConstants.*;

import java.util.Map;

import org.photonvision.PhotonUtils;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
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
    private final PositionVoltage pos = new PositionVoltage(0, 0, false, 0, 0, false, false, false);
    private final TrapezoidProfile profile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration)
    );
    
    private MotorOutputConfigs talonMfg() {
        MotorOutputConfigs mfg = new MotorOutputConfigs();
        mfg.NeutralMode = NeutralModeValue.Brake;
        mfg.Inverted = InvertedValue.Clockwise_Positive;
        return mfg;
    }

    private TalonFXConfiguration talonCfg(int cancoderID) {
        TalonFXConfiguration talon_cfg = new TalonFXConfiguration();

        FeedbackConfigs fuse_talon = talon_cfg.Feedback;
        fuse_talon.FeedbackRemoteSensorID = cancoderID;
        fuse_talon.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        fuse_talon.RotorToSensorRatio = gearRatio;
        fuse_talon.SensorToMechanismRatio = 1.0;

        Slot0Configs slot0 = talon_cfg.Slot0;
        slot0.GravityType = GravityTypeValue.Arm_Cosine;
        slot0.kP = kP; slot0.kI = kI; slot0.kD = kD;
        slot0.kV = kV; slot0.kS = kS; slot0.kG = kG;
        return talon_cfg;
    }

    private MagnetSensorConfigs encoderCfg() {
        MagnetSensorConfigs encoder_cfg = new MagnetSensorConfigs();
        encoder_cfg.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        encoder_cfg.MagnetOffset = encoderMagnentOffset;
        return encoder_cfg;
    }

    private void generateAngleMap() {
        double[][] angles = {
            {16+38.5,     58.99},
            {16+12+38.5,  52.77},
            {16+24+38.5,  47.89},
            {16+36+38.5,  45.67},
            {16+48+38.5,  42.58},
            {16+60+38.5,  41.00},
            {16+72+38.5,  38.70},
            {16+84+38.5,  38.075},
            {16+96+38.5,  37.00},
            {16+108+38.5, 35.59},
        };
        
        for (double[] pair: angles) {
            angleMap.put(Units.inchesToMeters(pair[0]), pair[1]);
        };
    }
    
    public ShooterPivot() {
        generateAngleMap();
        shooterPivotEncoder.getConfigurator().apply(encoderCfg());
        shooterPivot.getConfigurator().apply(talonCfg(shooterPivotEncoder.getDeviceID()));
        shooterPivot.getConfigurator().apply(talonMfg());
    }
    
    public double getAngle() {
        return shooterPivot.getPosition().getValueAsDouble();
    }

    public Command setAngle(double goal) {
        TrapezoidProfile.State m_goal = new TrapezoidProfile.State(goal, 0);
        TrapezoidProfile.State curPoint = new TrapezoidProfile.State(getAngle(), shooterPivot.getVelocity().getValueAsDouble());
        

        return run(() -> {
            TrapezoidProfile.State calc = profile.calculate(0.020, curPoint, m_goal);
            shooterPivot.setControl(
                pos.withPosition(calc.position).withVelocity(calc.velocity)
            );
        });
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
        return setAngle(Units.degreesToRotations(shuffleAngle.getDouble(0)));
    }
    
    public double getShotAngle(Pose3d goalPose) {
        return angleMap.get(PhotonUtils.getDistanceToPose(RobotContainer.drivetrain.getState().Pose, goalPose.toPose2d()));
    }

    public Command autoAngle(Pose3d goalPose3d) {
        return setAngle(Units.degreesToRotations(getShotAngle(goalPose3d)));
    }

    public Command stopMotors() {
        return runOnce(() -> shooterPivot.set(0));
    }

    double tolerance = Units.degreesToRotations(2.0);
    public boolean atSetpoint() {
        return Math.abs(shooterPivot.getClosedLoopError().getValue()) < tolerance;
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