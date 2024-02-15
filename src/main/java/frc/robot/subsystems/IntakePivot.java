package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import com.revrobotics.CANSparkBase.IdleMode;

public class IntakePivot implements Subsystem {
    private static int intakePivotID = 52;
    static CANSparkMax intakePivot;
    private SparkPIDController intakePivotPID;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

    public void IntakePivotInit() {
        // zero the motor
        intakePivot = new CANSparkMax(intakePivotID, MotorType.kBrushless);
        intakePivot.getEncoder().setPosition(0);

        intakePivot.setIdleMode(IdleMode.kBrake);

        // PID STUFF
        intakePivotPID = intakePivot.getPIDController();

        // coefficients
        kP = 0.1; 
        kI = 1e-4;
        kD = 1; 
        kIz = 0; 
        kFF = 0; 
        kMaxOutput = 1; 
        kMinOutput = -1;

        intakePivotPID.setP(kP);
        intakePivotPID.setI(kI);
        intakePivotPID.setD(kD);
        intakePivotPID.setIZone(kIz);
        intakePivotPID.setFF(kFF);
        intakePivotPID.setOutputRange(kMinOutput, kMaxOutput);
    }

    public double getAngle() {
        return intakePivot.getEncoder().getPosition();
    }

    public Command setAngle(double angle) {
        return run(() -> intakePivotPID.setReference(angle, CANSparkMax.ControlType.kPosition));
    }
}
