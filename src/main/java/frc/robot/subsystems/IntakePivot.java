package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import static frc.robot.Constants.IntakeConstants.*;

public class IntakePivot implements Subsystem {
    private final CANSparkMax intakePivot = new CANSparkMax(intakePivotID, MotorType.kBrushless);
    private final Encoder intakePivotEncoder = new Encoder(0, 1);

    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(6, 1);
    private final ProfiledPIDController controller = new ProfiledPIDController(kP, kI, kD, constraints, kDt);
    private final ArmFeedforward feedforward = new ArmFeedforward(kS, kG, kV);

    // __init__()
    public IntakePivot() {

        // set idle to brake and zero the motor

        intakePivot.setIdleMode(IdleMode.kBrake); 
    }

    public double getAngle() {
        return intakePivot.getEncoder().getPosition();
    }

    public Command setAngleCommand(double goal) {
        controller.setGoal(goal);
        return run(() -> intakePivot.setVoltage(controller.calculate(intakePivotEncoder.getDistance()) + feedforward.calculate(intakePivotEncoder.getDistance(), controller.getSetpoint().velocity)));
    }
}