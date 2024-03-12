package frc.robot.subsystems;

import static frc.robot.Constants.ShooterFlywheelConstants.*;

import java.util.Map;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class ShooterFlywheel implements Subsystem {
    private final TalonFX shooterFlywheelLeft = new TalonFX(shooterFlywheelLeftID, "canivore");
    private final TalonFX shooterFlywheelRight = new TalonFX(shooterFlywheelRightID, "canivore");
    private final VelocityVoltage fastMMConfig = new VelocityVoltage(Fast.shooterRPM / 60.0, Fast.shooterAcceleration, false, 0, 0, false, false, false);
    private final VelocityVoltage slowMMConfig = new VelocityVoltage(Slow.shooterRPM / 60.0, Slow.shooterAcceleration, true, 0, 1, false, false, false);

    private void configureMotors() {
        var mfg = new MotorOutputConfigs();
        mfg.NeutralMode = NeutralModeValue.Coast;

        var slot0 = new Slot0Configs();
        slot0.kP = Fast.kP; slot0.kI = Fast.kI; slot0.kD = Fast.kD;
        slot0.kV = Fast.kV; slot0.kS = Fast.kS; slot0.kA = Fast.kA;

        var slot1 = new Slot1Configs();
        slot1.kP = Slow.kP; slot1.kI = Slow.kI; slot1.kD = Slow.kD;
        slot1.kS = Slow.kS;

        shooterFlywheelLeft.getConfigurator().apply(slot0);
        shooterFlywheelLeft.getConfigurator().apply(slot1);
        shooterFlywheelLeft.getConfigurator().apply(mfg.withInverted(InvertedValue.CounterClockwise_Positive));
        
        shooterFlywheelRight.getConfigurator().apply(slot0);
        shooterFlywheelRight.getConfigurator().apply(slot1);
        shooterFlywheelRight.getConfigurator().apply(mfg.withInverted(InvertedValue.Clockwise_Positive));

        // SignalLogger.start();
    }

    public ShooterFlywheel() {
        configureMotors();
    }

    public Command setSpeed(double speedRPM) {
        upToSpeed = false;
        return run(() -> {
            System.out.println(shooterFlywheelLeft.getVelocity().getValueAsDouble()*60 + " | " + shooterFlywheelRight.getVelocity().getValueAsDouble()*60);
            shooterFlywheelLeft.setControl(fastMMConfig.withVelocity(speedRPM/60));
            shooterFlywheelRight.setControl(fastMMConfig.withVelocity(speedRPM/60));
        });
    }

    public Command slowFlywheels(double speedRPM) {
        return run(() -> {
            System.out.println(shooterFlywheelLeft.getVelocity().getValueAsDouble()*60 + " | " + shooterFlywheelRight.getVelocity().getValueAsDouble()*60);
            shooterFlywheelLeft.setControl(slowMMConfig.withVelocity(speedRPM/60));
            shooterFlywheelRight.setControl(slowMMConfig.withVelocity(speedRPM/60)); }
        );
    }
    
    Boolean upToSpeed = false;
    public Command setSpeedWithStop(double speedRPM) {
        upToSpeed = false;
        return setSpeed(speedRPM).until(() -> {
            if (shooterFlywheelLeft.getVelocity().getValueAsDouble() * 60 > (6050)) {
                upToSpeed = true;
            }
            if (upToSpeed && (shooterFlywheelLeft.getVelocity().getValueAsDouble() * 60) < 5750) {
                upToSpeed = false;
                return true;
            }
            return false;
        }).andThen(stopFlywheels());
    }
    public Command startFlywheels() {
        return setSpeedWithStop(6350);
    }

    private final GenericEntry shuffleFlywheelRPM = Shuffleboard.getTab("FieldInfo")
        .add("Shooter Rpm", 1000)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of(
            "min", 0,
            "max", 6000
        ))
        .getEntry();

    public Command setSpeedFromShuffle() {
        return setSpeedWithStop(shuffleFlywheelRPM.getDouble(1000));
    }

    public Command stopFlywheels() {
        return runOnce(() -> {
            upToSpeed = false;
            shooterFlywheelLeft.setVoltage(0);
            shooterFlywheelRight.setVoltage(0);
        });
    }


    // SysID Routine
    // SysIdRoutine routine = new SysIdRoutine(
    //     new SysIdRoutine.Config(null, Volts.of(4), null, (state) -> SignalLogger.writeString("state", state.toString())),
    //     new SysIdRoutine.Mechanism(
    //         (Measure<Voltage> volts) -> shooterFlywheelLeft.setControl(new VoltageOut(volts.in(Volts))), // Set voltage method
    //         null, // Log function
    //         this
    //     )
    // );

    // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    //     return routine.quasistatic(direction);
    // }
    // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    //     return routine.dynamic(direction);
    // }
}