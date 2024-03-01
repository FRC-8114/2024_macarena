package frc.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ShooterRotationConstants.*;

import org.photonvision.PhotonUtils;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class ShooterPivot implements Subsystem {
    private InterpolatingDoubleTreeMap angleMap = new InterpolatingDoubleTreeMap();
    
    private final CANSparkMax shooterPivot = new CANSparkMax(shooterPivotID, MotorType.kBrushless);
    private final CANcoder shooterPivotEncoder = new CANcoder(4, "canivore");

    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(5, 6);
    private final ProfiledPIDController controller = new ProfiledPIDController(kP, kI, kD, constraints, kDt);
    private final ArmFeedforward feedforward = new ArmFeedforward(kS, kG, kV);

    
    public ShooterPivot() {
        shooterPivot.setIdleMode(IdleMode.kBrake);
        shooterPivotEncoder.getConfigurator().apply(new MagnetSensorConfigs().withSensorDirection(SensorDirectionValue.Clockwise_Positive).withMagnetOffset(0.4287109375));

        // TODO: Generate test data
        angleMap.put(5.0, 20.0);
    }

    public double getAngle() {
        return shooterPivot.getEncoder().getPosition();
    }

    public Command setAngleCommand(double goal) {
        return run(() -> {
            shooterPivot.setVoltage(-(controller.calculate(shooterPivotEncoder.getPosition().getValueAsDouble(), goal)
                    + feedforward.calculate(shooterPivotEncoder.getPosition().getValueAsDouble(), controller.getSetpoint().velocity)));
        });
    }

    public Command stopMotors() {
        return runOnce(() -> shooterPivot.setVoltage(0));
    }

    public Command autoAngle(double distance) {
        return setAngleCommand(angleMap.get(distance));
    }

    public Command autoAngleFromPose(Pose2d currentPose, Pose3d goalPose) {
        return autoAngle(PhotonUtils.getDistanceToPose(currentPose, goalPose.toPose2d()));
    }

    public Command printEncoder() {
        return run(() -> System.out.println(Units.rotationsToDegrees(shooterPivotEncoder.getPosition().getValueAsDouble()) + " | " + shooterPivotEncoder.getPosition().getValueAsDouble())).ignoringDisable(true);
        // return run(() -> SmartDashboard.putNumber("Shooter Pivot Position", shooterPivotEncoder.getPosition().getValueAsDouble()));
    }


    // SysID
    private final MutableMeasure<Voltage> appliedVoltage = mutable(Volts.of(0));
    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    private final MutableMeasure<Angle> distance = mutable(Rotations.of(0));
    // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    private final MutableMeasure<Velocity<Angle>> velocity = mutable(RotationsPerSecond.of(0));

    public void consumeSysIdVoltage(Measure<Voltage> voltage) {
        shooterPivot.setVoltage(voltage.in(Volts));
    }

    private void consumeSysIdLog(SysIdRoutineLog log) {
        log.motor("Shooter Pivot")
            .voltage(
                appliedVoltage.mut_replace(
                    shooterPivot.getAppliedOutput() * shooterPivot.getBusVoltage(), Volts))
            .angularPosition(distance.mut_replace(shooterPivotEncoder.getPosition().getValueAsDouble(), Rotations))
            .angularVelocity(
                velocity.mut_replace(shooterPivotEncoder.getVelocity().getValueAsDouble(), RotationsPerSecond));
    }

    SysIdRoutine routine = new SysIdRoutine(
        new SysIdRoutine.Config(Volts.of(0.5).per(Seconds), Volts.of(2), null),
        new SysIdRoutine.Mechanism(
            this::consumeSysIdVoltage, // Set voltage method
            this::consumeSysIdLog, // Log function
            this
        )
    );

    private double upper_bound = Units.degreesToRotations(60);
    private double lower_bound = Units.degreesToRotations(0);

    public Command sysIdQuasistaticForward() {
        return routine.quasistatic(Direction.kForward).until(() -> (shooterPivotEncoder.getPosition().getValueAsDouble() <= lower_bound)).andThen(stopMotors());
    }

    public Command sysIdQuasistaticReverse() {
        return routine.quasistatic(Direction.kReverse).until(() -> (shooterPivotEncoder.getPosition().getValueAsDouble() >= upper_bound)).andThen(stopMotors());
    }

    public Command sysIdDynamicForward() {
        return routine.dynamic(Direction.kForward).until(() -> (shooterPivotEncoder.getPosition().getValueAsDouble() <= lower_bound)).andThen(stopMotors());
    }

    public Command sysIdDynamicReverse() {
        return routine.dynamic(Direction.kReverse).until(() -> (shooterPivotEncoder.getPosition().getValueAsDouble() >= upper_bound)).andThen(stopMotors());
    }
}