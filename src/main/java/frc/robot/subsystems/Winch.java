package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import static frc.robot.Constants.WinchConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.IdleMode;

public class Winch implements Subsystem {
    private final CANSparkMax winch = new CANSparkMax(winchID, MotorType.kBrushless);

    public Winch() {
        winch.setIdleMode(IdleMode.kBrake);
    }

    public Command setSpeedCommand(double voltage) {
        return run(() -> winch.setVoltage(voltage));
    }
}