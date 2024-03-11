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
    private final VelocityVoltage mmConfig = new VelocityVoltage(shooterRPM / 60.0, 70, false, 0, 0, false, false, false);
    private final VelocityVoltage slowConfig = new VelocityVoltage(4, 4, true, 0, 1, false, false, false);
    private final GenericEntry shooteRPM = Shuffleboard.getTab("FieldInfo").add("Shooter Rpm", 1000).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 6000)).getEntry();


    private void configureMotors() {
        var mfg = new MotorOutputConfigs();
        mfg.NeutralMode = NeutralModeValue.Coast;

        var slot0 = new Slot0Configs();
        slot0.kP = kP; slot0.kI = kI; slot0.kD = kD;
        slot0.kV = kV; slot0.kS = kS; slot0.kA = kA;

        var slot1 = new Slot1Configs();
        slot1.kP = 30; slot1.kI = 0.2; slot1.kD = 9; slot1.kS = 0.2;

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

    Boolean upToSpeed = false;

    public Command setSpeed(double speedRPM) {
        upToSpeed = false;
        return run(() -> {
            // System.out.println(shooterFlywheelLeft.getVelocity().getValueAsDouble()*60 +
            // " | " + shooterFlywheelRight.getVelocity().getValueAsDouble()*60);
            shooterFlywheelLeft.setControl(mmConfig.withVelocity(speedRPM / 60));
            shooterFlywheelRight.setControl(mmConfig.withVelocity((speedRPM) / 60));
        }).until(() -> {
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

    public Command slowFlywheels(double rpm) {
        return run(() -> {
            System.out.println(shooterFlywheelLeft.getVelocity().getValueAsDouble()*60 + " | " + shooterFlywheelRight.getVelocity().getValueAsDouble()*60);
            shooterFlywheelLeft.setControl(slowConfig.withVelocity(rpm/60));
            shooterFlywheelRight.setControl(slowConfig.withVelocity(rpm/60)); }
        );
    }

    public Command setSpeedNoStop(double speedRPM) {
        upToSpeed = false;
        return run(() -> {
            System.out.println(shooterFlywheelLeft.getVelocity().getValueAsDouble()*60 + " | " + shooterFlywheelRight.getVelocity().getValueAsDouble()*60);
            shooterFlywheelLeft.setControl(mmConfig.withVelocity(speedRPM/60));
            shooterFlywheelRight.setControl(mmConfig.withVelocity((speedRPM)/60));
        });
    }

    public Command startFlywheels() {
        return setSpeed(6350);
    };

    public Command setSpeedFromShuffle() {
        return setSpeed(shooteRPM.getDouble(1000));
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