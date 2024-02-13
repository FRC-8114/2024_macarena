package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

public class IntakePivot {
    private static int intakePivotID = 0;
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

    public double GetAngle() {
        return intakePivot.getEncoder().getPosition();
    }
}
