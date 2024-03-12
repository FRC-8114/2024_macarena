package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import static frc.robot.Constants.TelescopeConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.IdleMode;

public class Telescope implements Subsystem {
    private final CANSparkMax telescopeLeft = new CANSparkMax(telescopeIDLeft, MotorType.kBrushless);
    private final CANSparkMax telescopeRight = new CANSparkMax(telescopeIDRight, MotorType.kBrushless);

    public Telescope() {
        telescopeRight.setIdleMode(IdleMode.kBrake);
        telescopeLeft.setIdleMode(IdleMode.kBrake);

        telescopeRight.follow(telescopeLeft);
    }

    public Command setSpeedCommand(double voltage) {
        return runOnce(() -> telescopeLeft.setVoltage(voltage));
    }
}