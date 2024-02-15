package frc.robot;

public class Constants {
    // Shooter Constants
    public static final class ShooterConstants {
        public static final int shooterPivotID = 60;
        public static final int shooterFlywheelLeftID = 61;
        public static final int shooterFlywheelRightID = 62;
    }

    // Telescope Constants
    public static final class TelescopeConstants {
        public static final int telescopeID = 59;
    }
    
    // Intake Constants
    public static final class IntakeConstants {
        public static final int intakePivotID = 52;
        public static final int intakeRollersID = 51;

        // intake PID coefficients
        public static final double kP = 0.1;
        public static final double kI = 1e-4;
        public static final double kD = 1;
        public static final double kIz = 0;
        public static final double kFF = 0;
        public static final double kMaxOutput = 1;
        public static final double kMinOutput = -1;
    };
}
