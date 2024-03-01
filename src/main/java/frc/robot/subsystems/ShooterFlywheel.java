package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ShooterFlywheelConstants.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class ShooterFlywheel implements Subsystem {
    private final TalonFX shooterFlywheelLeft;
    private final TalonFX shooterFlywheelRight;
    private final MotionMagicVelocityVoltage mmConfig = new MotionMagicVelocityVoltage(shooterRPM / 60.0, 50, false, 3, 0, false, false, false);
    

    private void configureMotors() {
        var mfg = new MotorOutputConfigs();
        mfg.NeutralMode = NeutralModeValue.Coast;

        var cfg = new Slot0Configs();
        cfg.kP = kP; cfg.kI = kI; cfg.kD = kD; cfg.kV = kV; cfg.kS = kS;

        shooterFlywheelLeft.getConfigurator().apply(cfg);
        shooterFlywheelLeft.getConfigurator().apply(mfg.withInverted(InvertedValue.CounterClockwise_Positive));
        
        shooterFlywheelRight.getConfigurator().apply(cfg, 0.050);
        shooterFlywheelRight.getConfigurator().apply(mfg);

        shooterFlywheelLeft.optimizeBusUtilization();
        shooterFlywheelRight.optimizeBusUtilization();

        //SignalLogger.start();
    }

    public ShooterFlywheel() {
        shooterFlywheelLeft = new TalonFX(shooterFlywheelLeftID, "canivore");
        shooterFlywheelRight = new TalonFX(shooterFlywheelRightID, "canivore");
        configureMotors();
    }

    public Command setSpeedCommand(double speedRPS) {
        return run(() -> {
            // shooterFlywheelLeft.sshooterMagicVelocityetVoltage(voltage);
            // shooterFlywheelRight.setVoltage(voltage);
            shooterFlywheelLeft.setControl(mmConfig.withVelocity(speedRPS));
            shooterFlywheelRight.setControl(mmConfig.withVelocity(speedRPS));
        });
    }
    public Command startFlywheels() {
        return setSpeedCommand(67);
    };

    public Command stopFlywheels() {
        return runOnce(() -> {
            shooterFlywheelLeft.setVoltage(0);
            shooterFlywheelRight.setVoltage(0);
        });
    }


    // SysID Routine
    SysIdRoutine routine = new SysIdRoutine(
        new SysIdRoutine.Config(null, Volts.of(9), null, (state) -> SignalLogger.writeString("Flywheel State", state.toString())),
        new SysIdRoutine.Mechanism(
            this::consumeSysIdVoltage, // Set voltage method
            null, // Log function
            this
        )
    );

    private void consumeSysIdVoltage(Measure<Voltage> voltage) {
        shooterFlywheelLeft.setVoltage(voltage.in(Volts));
        shooterFlywheelRight.setVoltage(voltage.in(Volts));
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }
}