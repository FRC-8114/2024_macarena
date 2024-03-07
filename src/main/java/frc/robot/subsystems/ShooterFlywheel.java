package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ShooterFlywheelConstants.*;

import java.util.Map;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class ShooterFlywheel implements Subsystem {
    private final TalonFX shooterFlywheelLeft = new TalonFX(shooterFlywheelLeftID, "canivore");
    private final TalonFX shooterFlywheelRight = new TalonFX(shooterFlywheelRightID, "canivore");
    private final VelocityVoltage mmConfig = new VelocityVoltage(shooterRPM / 60.0, 70, false, 0, 0, false, false, false);
    private final GenericEntry shooteRPM = Shuffleboard.getTab("FieldInfo").add("Shooter Rpm", 1000).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 6000)).getEntry();


    private void configureMotors() {
        var mfg = new MotorOutputConfigs();
        mfg.NeutralMode = NeutralModeValue.Coast;

        var cfg = new Slot0Configs();
        cfg.kP = kP; cfg.kI = kI; cfg.kD = kD; cfg.kV = kV; cfg.kS = kS; cfg.kA = kA;

        shooterFlywheelLeft.getConfigurator().apply(cfg);
        shooterFlywheelLeft.getConfigurator().apply(mfg.withInverted(InvertedValue.CounterClockwise_Positive));
        
        shooterFlywheelRight.getConfigurator().apply(cfg, 0.050);
        shooterFlywheelRight.getConfigurator().apply(mfg.withInverted(InvertedValue.Clockwise_Positive));

        // SignalLogger.start();
    }

    public ShooterFlywheel() {
        configureMotors();
    }


    Boolean upToSpeed = false;
    public Command setSpeedCommand(double speedRPM) {
        upToSpeed = false;
        return run(() -> {
            // shooterFlywheelLeft.sshooterMagicVelocityetVoltage(voltage);
            // shooterFlywheelRight.setVoltage(voltage);
            // System.out.println(shooterFlywheelLeft.getVelocity().getValueAsDouble()*60 + " | " + shooterFlywheelRight.getVelocity().getValueAsDouble()*60);
            shooterFlywheelLeft.setControl(mmConfig.withVelocity(speedRPM/60));
            shooterFlywheelRight.setControl(mmConfig.withVelocity((speedRPM)/60));
        }).until(() -> { if(shooterFlywheelLeft.getVelocity().getValueAsDouble()*60 > (6100)) {
            upToSpeed = true; }
            if (upToSpeed && (shooterFlywheelLeft.getVelocity().getValueAsDouble()*60) < 5750) {
                upToSpeed = false;
                return true;
            }
                return false;
        }).andThen(stopFlywheels());
    }
    public Command startFlywheels() {
        return setSpeedCommand(6350);
    };

    public Command setSpeedFromShuffle() {
        return setSpeedCommand(shooteRPM.getDouble(1000));
    }

    public Command stopFlywheels() {
        return runOnce(() -> {
            shooterFlywheelLeft.setVoltage(0);
            shooterFlywheelRight.setVoltage(0);
        });
    }


    // SysID Routine
    SysIdRoutine routine = new SysIdRoutine(
        new SysIdRoutine.Config(null, Volts.of(4), null, (state) -> SignalLogger.writeString("state", state.toString())),
        new SysIdRoutine.Mechanism(
            (Measure<Voltage> volts) -> shooterFlywheelLeft.setControl(new VoltageOut(volts.in(Volts))), // Set voltage method
            null, // Log function
            this
        )
    );

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }
}