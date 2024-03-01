package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.MutableMeasure.mutable;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import static frc.robot.Constants.IntakeConstants.*;

public class IntakePivot implements Subsystem {
    private final CANSparkMax intakePivot = new CANSparkMax(intakePivotID, MotorType.kBrushless);
    private final Encoder intakePivotEncoder = new Encoder(8, 9);

    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(6, 1);
    private final ProfiledPIDController controller = new ProfiledPIDController(kP, kI, kD, constraints, kDt);
    private final ArmFeedforward feedforward = new ArmFeedforward(kS, kG, kV);

    public IntakePivot() {
        intakePivot.setIdleMode(IdleMode.kBrake);
        intakePivotEncoder.setDistancePerPulse(1.0/2048.0);
        intakePivotEncoder.setMinRate(1);
        intakePivotEncoder.setSamplesToAverage(5);
    }

    public double getAngle() {
        return intakePivotEncoder.getDistance();
    }

    public Command setAngleCommand(double goal) {
        controller.setGoal(goal);
        return run(() -> {
            intakePivot.setVoltage(controller.calculate(intakePivotEncoder.getDistance())
                    + feedforward.calculate(intakePivotEncoder.getDistance(), controller.getSetpoint().velocity));
        });
    }

    public Command stopMotor() {
        return runOnce(() -> intakePivot.setVoltage(0));
    }

    public Command printEncoder() {
        return run(() -> System.out.println(intakePivotEncoder.getDistance()));
    }

    // SysID
    private final MutableMeasure<Voltage> appliedVoltage = mutable(Volts.of(0));
    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    private final MutableMeasure<Angle> distance = mutable(Rotations.of(0));
    // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    private final MutableMeasure<Velocity<Angle>> velocity = mutable(RotationsPerSecond.of(0));

    public void sysIdVoltage(Measure<Voltage> voltage) {
        intakePivot.setVoltage(voltage.in(Volts));
    }

    private void sysIdLog(SysIdRoutineLog log) {
        log.motor("Intake Pivot")
            .voltage(
                appliedVoltage.mut_replace(
                    intakePivot.get() * RobotController.getBatteryVoltage(), Volts))
            .angularPosition(distance.mut_replace(intakePivotEncoder.getDistance(), Rotations))
            .angularVelocity(
                velocity.mut_replace(intakePivotEncoder.getRate(), RotationsPerSecond));
    }

    SysIdRoutine routine = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            this::sysIdVoltage, // Set voltage function
            this::sysIdLog, // Log function
            this
        )
    );

    private double upper_bound = Units.degreesToRotations(62);
    private double lower_bound = Units.degreesToRotations(-6);

    public Command sysIdQuasistaticForward() {
        return routine.quasistatic(Direction.kForward).until(() -> (intakePivotEncoder.getDistance() <= lower_bound)).andThen(stopMotor());
    }

    public Command sysIdQuasistaticReverse() {
        return routine.quasistatic(Direction.kReverse).until(() -> (intakePivotEncoder.getDistance() >= upper_bound)).andThen(stopMotor());
    }

    public Command sysIdDynamicForward() {
        return routine.dynamic(Direction.kForward).until(() -> (intakePivotEncoder.getDistance() <= lower_bound)).andThen(stopMotor());
    }

    public Command sysIdDynamicReverse() {
        return routine.dynamic(Direction.kReverse).until(() -> (intakePivotEncoder.getDistance() >= upper_bound)).andThen(stopMotor());
    }
}