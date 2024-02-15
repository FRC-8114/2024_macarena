package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import static frc.robot.Constants.IntakeConstants.*;

public class IntakePivot extends SubsystemBase {
    private final CANSparkMax intakePivot;
    private final SparkPIDController intakePivotPID;

    // __init__()
    public IntakePivot() {
        intakePivot = new CANSparkMax(intakePivotID, MotorType.kBrushless);

        // set idle to brake and zero the motor

        intakePivot.setIdleMode(IdleMode.kBrake);
        intakePivot.getEncoder().setPosition(0);

        intakePivotPID = intakePivot.getPIDController();

        intakePivotPID.setP(kP);
        intakePivotPID.setI(kI);
        intakePivotPID.setD(kD);
        intakePivotPID.setIZone(kIz);
        intakePivotPID.setFF(kFF);
        intakePivotPID.setOutputRange(kMaxOutput, kMinOutput);
    }

    public double getAngle() {
        return intakePivot.getEncoder().getPosition();
    }

    public Command setAngleCommand(double angle) {
        return run(() -> intakePivotPID.setReference(angle, ControlType.kPosition));
    }
}