package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import com.revrobotics.CANSparkMax;

import static frc.robot.Constants.IntakeConstants.*;

public class IntakeRollers implements Subsystem {
    final static CANSparkMax intakeRollers = new CANSparkMax(
        intakeRollersID,
        MotorType.kBrushless
    );
    final DigitalInput limSwitch = new DigitalInput(limSwitchDIO);

    public IntakeRollers() {
        intakeRollers.setIdleMode(IdleMode.kBrake);
    }

    public Command intakeNote() {
        return run(() -> intakeRollers.setVoltage(11))
            .until(() -> limSwitch.get())
            .andThen(intakeStop());
    }

    public Command outtakeNote() {
        return run(() -> intakeRollers.setVoltage(-11))
            .withTimeout(1)
            .andThen(intakeStop());
    }

    public Command intakeStop() {
        return runOnce(() -> intakeRollers.setVoltage(0));
    }

}
