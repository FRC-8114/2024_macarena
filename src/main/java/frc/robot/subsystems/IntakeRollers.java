package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import com.revrobotics.CANSparkMax;

import static frc.robot.Constants.IntakeConstants.*;

public class IntakeRollers implements Subsystem {
    final static CANSparkMax intakeRollers = new CANSparkMax(intakeRollersID, MotorType.kBrushless);

    public IntakeRollers() {
        intakeRollers.setIdleMode(IdleMode.kBrake);
    }

    public Command IntakeNote(double voltage) {
        return run(() -> intakeRollers.setVoltage(voltage));
    }

    public Command IntakeReverse(double voltage) {
        return run(() -> intakeRollers.setVoltage(-voltage));
    }

    public Command IntakeStop() {
        return runOnce(() -> intakeRollers.setVoltage(0));
    }
}
