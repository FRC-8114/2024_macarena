package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import static frc.robot.Constants.TelescopeConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.IdleMode;

public class Telescope implements Subsystem {
    private final CANSparkMax telescope = new CANSparkMax(telescopeID, MotorType.kBrushless);

    public Telescope() {
        telescope.setIdleMode(IdleMode.kBrake);
    }

    public Command setSpeedCommand(double voltage) {
        return runOnce(() -> telescope.setVoltage(voltage));
    }
}