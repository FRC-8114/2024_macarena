package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import static frc.robot.Constants.TrapConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.IdleMode;

public class Trap implements Subsystem {
    private final CANSparkMax trap = new CANSparkMax(trapID, MotorType.kBrushless);

    public Trap() {
        trap.setIdleMode(IdleMode.kBrake);
    }

    public Command setVoltageCommand(double voltage) {
        return run(() -> trap.setVoltage(voltage));
    }
}