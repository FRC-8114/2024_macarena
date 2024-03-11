package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import com.revrobotics.CANSparkMax;

import static frc.robot.Constants.IntakeConstants.*;


public class IntakeRollers extends SubsystemBase {
    final static CANSparkMax intakeRollers = new CANSparkMax(
        intakeRollersID,
        MotorType.kBrushless
    );
    final DigitalInput limSwitch = new DigitalInput(limSwitchDIO);

    private boolean noteIntaked = false;
    private Timer currentHit = new Timer();

    public IntakeRollers() {
        intakeRollers.setIdleMode(IdleMode.kBrake);
        intakeRollers.setOpenLoopRampRate(0.35);
    }

    public Command intakeNote() {
        return run(() -> intakeRollers.setVoltage(-10))
            .until(() -> limSwitch.get() || intakeRollers.getOutputCurrent() > 72.0)
            .andThen(intakeStop());
    }

    public Command outtakeNote() {
        return run(() -> intakeRollers.setVoltage(12))
            .withTimeout(.5)
            .andThen(intakeStop());
    }

    public Command slowOuttakeNote() {
        return run(() -> intakeRollers.setVoltage(4.75))
            .withTimeout(1)
            .andThen(intakeStop());
    }

    public Command intakeStop() {
        return runOnce(() -> intakeRollers.setVoltage(0));
    }

    public Command printLim() {
        return runOnce(() -> System.out.println("" + limSwitch.get()));
    }

    @Override
    public void periodic()
    {
        
        // SmartDashboard.putNumber("Intake CUrrent", intakeRollers.getOutputCurrent());
    }

}
