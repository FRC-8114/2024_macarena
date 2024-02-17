package frc.robot;

import java.util.function.DoubleSupplier;

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
        public static double kDt = 0.02;
        public static double kMaxVelocity = 1.75;
        public static double kMaxAcceleration = 0.75;
        public static double kP = 1.3;
        public static double kI = 0.0;
        public static double kD = 0.7;
        public static double kS = 1.1;
        public static double kG = 1.2;
        public static double kV = 1.3;
    };
}
