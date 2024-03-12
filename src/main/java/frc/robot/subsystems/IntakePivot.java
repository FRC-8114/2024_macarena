package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.*;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakePivot extends SubsystemBase {
    private final CANSparkMax intakePivot = new CANSparkMax(intakePivotID, MotorType.kBrushless);
    //private final Encoder intakePivotEncoder = new Encoder(8, 9);
    private final RelativeEncoder intakePivotEncoder = intakePivot.getEncoder();
    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration);
    private final ProfiledPIDController controller = new ProfiledPIDController(kP, kI, kD, constraints, kDt);
    private final ArmFeedforward feedforward = new ArmFeedforward(kS, kG, kV);

    public IntakePivot() {
        intakePivot.setIdleMode(IdleMode.kBrake);
        intakePivotEncoder.setPosition(0);
        intakePivotEncoder.setPositionConversionFactor(1.0/30.0);
        controller.setTolerance(Units.degreesToRotations(2.0));
    }

    public double getAngle() {
        return intakePivotEncoder.getPosition();
    }

    public Command setAngle(double goal) {
        return run(() -> {
            double curPose = Units.rotationsToRadians(intakePivotEncoder.getPosition());
            double ff = feedforward.calculate(curPose, controller.getSetpoint().velocity);
            intakePivot.setVoltage(controller.calculate(intakePivotEncoder.getPosition(), goal) + ff);
        });
    }

    public BooleanSupplier atSetpoint() {
        return () -> controller.atSetpoint() || intakePivot.getOutputCurrent() > 70;
    }

    public Command intakeSetVoltage(double voltage) {
        return run(() -> intakePivot.setVoltage(voltage));
    }

    public Command intakeDown() {
        return setAngle(0.39365264773368835)
            .until(atSetpoint())
            .andThen(stopMotor());
    }

    public Command intakeAmp() {
        return setAngle(0.17)
            .until(atSetpoint())
            .andThen(stopMotor());
    }

    public Command intakeUp() {
        return setAngle(0)
            .until(atSetpoint())
            .andThen(stopMotor());
    }

    public Command resetAngle() {
        return runOnce(() -> intakePivotEncoder.setPosition(0.0));
    }

    public Command stopMotor() {
        return runOnce(() -> intakePivot.setVoltage(0));
    }

    public Command printEncoder() {
        return run(() -> System.out.println(intakePivotEncoder.getPosition()));
    }

    // // SysID
    // private final MutableMeasure<Voltage> appliedVoltage = mutable(Volts.of(0));
    // // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    // private final MutableMeasure<Angle> distance = mutable(Rotations.of(0));
    // // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    // private final MutableMeasure<Velocity<Angle>> velocity = mutable(RotationsPerSecond.of(0));


    // private void sysIdLog(SysIdRoutineLog log) {
    //     log.motor("Intake Pivot")
    //         .voltage(
    //             appliedVoltage.mut_replace(
    //                 intakePivot.get() * RobotController.getBatteryVoltage(), Volts))
    //         .angularPosition(distance.mut_replace(intakePivotEncoder.getDistance(), Rotations))
    //         .angularVelocity(
    //             velocity.mut_replace(intakePivotEncoder.getRate(), RotationsPerSecond));
    // }

    // SysIdRoutine routine = new SysIdRoutine(
    //     new SysIdRoutine.Config(),
    //     new SysIdRoutine.Mechanism(
    //         (Measure<Voltage> volts) -> intakePivot.setVoltage(volts.in(Volts)), // Set voltage function
    //         this::sysIdLog, // Log function
    //         this
    //     )
    // );

    // private double upper_bound = Units.degreesToRotations(62);
    // private double lower_bound = Units.degreesToRotations(-6);

    // public Command sysIdQuasistaticForward() {
    //     return routine.quasistatic(Direction.kForward).until(() -> (intakePivotEncoder.getDistance() <= lower_bound)).andThen(stopMotor());
    // }

    // public Command sysIdQuasistaticReverse() {
    //     return routine.quasistatic(Direction.kReverse).until(() -> (intakePivotEncoder.getDistance() >= upper_bound)).andThen(stopMotor());
    // }

    // public Command sysIdDynamicForward() {
    //     return routine.dynamic(Direction.kForward).until(() -> (intakePivotEncoder.getDistance() <= lower_bound)).andThen(stopMotor());
    // }

    // public Command sysIdDynamicReverse() {
    //     return routine.dynamic(Direction.kReverse).until(() -> (intakePivotEncoder.getDistance() >= upper_bound)).andThen(stopMotor());
    // }
}