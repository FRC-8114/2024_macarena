package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;

import static frc.robot.Constants.IntakeConstants.*;

import java.util.function.BooleanSupplier;


public class IntakeRollers extends SubsystemBase {
    final static CANSparkMax intakeRollers = new CANSparkMax(
        intakeRollersID,
        MotorType.kBrushless
    );
    DigitalInput limSwitch = new DigitalInput(limSwitchDIO);
    DigitalInput limSwitch2 =  new DigitalInput(limSwitchDIO2);

    public IntakeRollers() {
        intakeRollers.setIdleMode(IdleMode.kBrake);
        intakeRollers.setOpenLoopRampRate(0.00);
    }

    public Command intakeNote() {
        return run(() -> intakeRollers.setVoltage(-6.5))
            .until(() -> (limSwitch.get() || limSwitch2.get()))
            .andThen(Commands.waitSeconds(0.025).andThen(() -> intakeRollers.setVoltage(0)));
    }

    public BooleanSupplier limBooleanSupplier() {
        return () -> (!limSwitch.get() || !limSwitch2.get());
    }

    public Command intakeNoteAmp() {
        return run(() -> intakeRollers.setVoltage(-8));
    }

    public Command outtakeNote() {

        return run(() -> {intakeRollers.set(1);})
            .withTimeout(.80)
            .andThen(this.intakeStop());
    }

    public Command outtakeConstant() {
        return run(() -> intakeRollers.setVoltage(12));
    }

    public Command slowOuttakeNote() {
        return run(() -> intakeRollers.setVoltage(2.35))
            .withTimeout(.5)
            .andThen(intakeStop());
    }

    public Command intakeStop() {
        return runOnce(() -> intakeRollers.setVoltage(0));
    }

    public Command printLim() {
        return runOnce(() -> System.out.println("" + limSwitch.get() + " | " + limSwitch2.get()));
    }
}
