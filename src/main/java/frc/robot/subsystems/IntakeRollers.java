package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
// import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.revrobotics.CANSparkMax;

public class IntakeRollers {
    private static int intakeRollersID = 0;
    final static CANSparkMax intakeRollers = new CANSparkMax(intakeRollersID, MotorType.kBrushless);

    public void IntakeInit() {
        intakeRollers.setIdleMode(IdleMode.kBrake);
    }

    public void IntakeNote(double voltage) {
        intakeRollers.setVoltage(voltage);
    }

    public void IntakeReverse(double voltage) {
        intakeRollers.setVoltage(-voltage);
    }
}
